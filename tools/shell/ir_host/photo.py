"""Photo transfer, CRC32 verification, RGB565 decoding and atomic PNG output."""
from __future__ import annotations
import hashlib, json, os, time, zlib
from pathlib import Path
from .errors import (ConnectionLost, DeviceCommandError, TransportError, NoPhotosError, PartialExecutionError,
                     ProtocolError, ReadTimeout)
from .protocol import ACK, NACK, CANCEL, PROMPT, PhotoHeader, PhotoListItem, is_cli_sync_response, strip_ansi

class PhotoEngine:
    def __init__(self, parser, config, log=print):
        self._stray=0
        self.parser=parser; self.transport=parser.transport; self.config=config; self.log=log
        self.output=Path(config.output_dir)
        self._last_recovery_ok = True

    def save_index(self, expected:int, items:list[PhotoListItem], skipped:int)->Path:
        self.output.mkdir(parents=True,exist_ok=True)
        target=self.output/"photo_index.json"; temp=target.with_suffix(".json.tmp")
        payload={"schema_version":1,"protocol":"IRPHOTOS v1","expected_entries":expected,
                 "returned_items":len(items),"skipped_items":skipped,
                 "photos":[{"event_id":x.event_id,"tick_ms":x.tick_ms,"width":x.width,"height":x.height,
                            "pixel_format":x.pixel_format,"bytes_per_pixel":x.bpp,"size_bytes":x.size,
                            "crc32":f"{x.crc32:08X}"} for x in items]}
        try:
            with temp.open("w",encoding="utf-8",newline="\n") as f:
                json.dump(payload,f,indent=2);f.write("\n");f.flush();os.fsync(f.fileno())
            os.replace(temp,target); return target
        except Exception:
            temp.unlink(missing_ok=True)
            raise

    @staticmethod
    def _file_sha256(path:Path)->str:
        digest=hashlib.sha256()
        with path.open("rb") as stream:
            for block in iter(lambda:stream.read(65536),b""):
                digest.update(block)
        return digest.hexdigest().upper()

    @staticmethod
    def _file_crc32(path:Path)->int:
        crc=0
        with path.open("rb") as stream:
            for block in iter(lambda:stream.read(65536),b""):
                crc=zlib.crc32(block,crc)
        return crc & 0xFFFFFFFF

    @staticmethod
    def rgb565_to_rgb(data:bytes, byte_order:str="big")->bytes:
        if len(data)%2: raise ValueError("RGB565 payload length must be even")
        out=bytearray(len(data)//2*3)
        for i in range(0,len(data),2):
            value=int.from_bytes(data[i:i+2],byte_order)
            r=(value>>11)&0x1f; g=(value>>5)&0x3f; b=value&0x1f
            j=(i//2)*3
            out[j]=(r<<3)|(r>>2); out[j+1]=(g<<2)|(g>>4); out[j+2]=(b<<3)|(b>>2)
        return bytes(out)

    def _paths(self,h:PhotoHeader):
        stem=f"event_{h.event_id}_{h.width}x{h.height}"
        return self.output/(stem+".png"), self.output/(stem+".rgb565"), self.output/(stem+".json")

    def _png_state(self,png:Path,width:int,height:int,crc:int)->str:
        """'match'   - valid PNG whose pixels re-encode to the RGB565 frame with this CRC32
                       (RGB565 -> RGB888 -> RGB565 is lossless; integer scaling is undone);
           'different' - valid image with other content;
           'invalid' - not a readable image (damaged or truncated file)."""
        try:
            from PIL import Image
            with Image.open(png) as im:
                im.load(); rgb=im.convert("RGB")
        except Exception:
            return "invalid"
        try:
            if rgb.width%width or rgb.height*width!=rgb.width*height: return "different"
            if rgb.width!=width: rgb=rgb.resize((width,height),Image.Resampling.NEAREST)
            px=rgb.tobytes(); out=bytearray(width*height*2)
            for i in range(width*height):
                r,g,b=px[3*i],px[3*i+1],px[3*i+2]
                v=((r>>3)<<11)|((g>>2)<<5)|(b>>3)
                hi,lo=v>>8,v&255
                if self.config.rgb565_byte_order=="big": out[2*i],out[2*i+1]=hi,lo
                else: out[2*i],out[2*i+1]=lo,hi
            return "match" if (zlib.crc32(bytes(out))&0xffffffff)==crc else "different"
        except Exception:
            return "different"

    def _png_matches(self,png:Path,width:int,height:int,crc:int)->bool:
        return self._png_state(png,width,height,crc)=="match"

    def _already_valid(self,item:PhotoListItem)->bool:
        png,raw,meta=self._paths(PhotoHeader(item.event_id,item.width,item.height,item.pixel_format,item.bpp,item.size,item.crc32,256))
        if self.config.overwrite or not png.exists(): return False
        if not self.config.keep_metadata:
            if self.config.keep_rgb565 and (not raw.exists() or raw.stat().st_size != item.size or self._file_crc32(raw)!=item.crc32): return False
            return self._png_matches(png,item.width,item.height,item.crc32)
        if not meta.exists(): return False
        if self.config.keep_rgb565 and (not raw.exists() or raw.stat().st_size != item.size): return False
        try:
            d=json.loads(meta.read_text(encoding="utf-8"))
            if int(d.get("crc32","-1"),16)!=item.crc32 or d.get("size_bytes")!=item.size: return False
            if d.get("png_sha256")!=self._file_sha256(png): return False
            if self.config.keep_rgb565 and self._file_crc32(raw)!=item.crc32: return False
            from PIL import Image
            with Image.open(png) as image:
                image.verify()
            with Image.open(png) as image:
                stored_scale=int(d.get("png_scale",1))
                return image.size==(item.width*stored_scale,item.height*stored_scale)
        except Exception: return False

    def _read_header(self, initial_line=None)->PhotoHeader:
        """Wait for the IRPHOTO4 header after `photo id N`.

        The firmware may answer with plain text followed by the prompt instead of
        a header (no stored photo, busy flash, not DISARMED ...). Such an answer is
        a final reply, not a protocol error: it is reported immediately, the prompt
        is consumed and nothing is retried or "recovered"."""
        if initial_line is not None:
            if initial_line.startswith("ERR:"): raise DeviceCommandError(initial_line)
            return PhotoHeader.parse(initial_line)
        deadline=time.monotonic()+self.config.command_timeout
        notes=[]
        while True:
            remaining=deadline-time.monotonic()
            if remaining<=0: raise ReadTimeout("timeout waiting for IRPHOTO4 header")
            raw,marker=self.transport.read_line_or_marker((PROMPT,),remaining)
            text=strip_ansi(raw).replace("\r","")
            if marker is not None and text.endswith(PROMPT.decode("ascii")): text=text[:-len(PROMPT)]
            line=text.strip()
            if marker is None:
                if line.startswith("IRPHOTO4,"): return PhotoHeader.parse(line)
                if line.lower().startswith("photo id "): continue      # echo of our command
                if line: notes.append(line)
                continue
            if line and not line.lower().startswith("photo id "): notes.append(line)
            message="\n".join(notes) or "the device returned to the prompt without starting the photo transfer"
            if any("no stored photo" in n for n in notes): raise NoPhotosError(message)
            raise DeviceCommandError(message)

    _PHOTO_TOKENS=(b"IRPHOTO_CHUNK,",b"IRPHOTO_END,",b"IRPHOTO_ABORT,",b"IRPHOTO_ERROR,")

    def _next_photo_record(self,timeout:float)->str:
        """Next protocol record of a photo stream, resynchronising after line noise.

        If a byte is lost on the UART, the line framing shifts: payload bytes show up
        where a text header is expected and the next header is glued to their tail.
        Binary junk is never printed; the record is located by its token anywhere in
        the received line and everything before it is counted and discarded."""
        deadline=time.monotonic()+timeout
        while True:
            remaining=deadline-time.monotonic()
            if remaining<=0: raise ReadTimeout("timeout waiting for photo record")
            try: raw=self.transport.read_line_bytes(remaining,max_bytes=4096)
            except (ReadTimeout,ConnectionLost): raise
            except TransportError:
                self._stray+=4096; continue          # overlong junk line, already consumed
            pos=max(raw.rfind(t) for t in self._PHOTO_TOKENS)
            if pos<0:
                self._stray+=len(raw); continue
            self._stray+=pos
            return raw[pos:].decode("ascii","replace").strip()

    def _recover(self, pending_payload:int=0)->bool:
        """Abort transfer and drain any already-committed chunk before text parsing."""
        self._last_recovery_ok=False
        try: self.transport.write(CANCEL)
        except Exception: return False
        if pending_payload:
            try: self.transport.read_exact(pending_payload,self.config.photo_timeout,idle_timeout=self.config.photo_timeout)
            except Exception: return False
        else:
            # A header/persistence error can occur after the firmware has put a
            # chunk header on the wire but before the host has parsed its length.
            # Probe one protocol line so we can drain that exact binary payload.
            try:
                line=self.transport.read_line(1.0)
            except Exception:
                line=""
            if line.startswith("IRPHOTO4,"):
                try: line=self.transport.read_line(1.0)
                except Exception: line=""
            if line.startswith("IRPHOTO_CHUNK,"):
                try:
                    fields=line.split(",")
                    length=int(fields[2])
                    if len(fields)==4 and 0<length<=4096:
                        self.transport.read_exact(length,self.config.photo_timeout,idle_timeout=self.config.photo_timeout)
                except Exception:
                    return False
            elif line.startswith(("IRPHOTO_ABORT,","IRPHOTO_END,","IRPHOTO_ERROR,")):
                try: self.parser.wait_prompt(3.0)
                except Exception: return False
                self._last_recovery_ok=True
                return True
        try:
            _response, marker=self.transport.read_until_any(
                (b"IRPHOTO_ABORT,",b"IRPHOTO_END,",b"IRPHOTO_ERROR,",b"IRPHOTO_ALL_ABORT,",b"IR> "),
                25.0)
            if marker != b"IR> ":
                try: self.transport.read_line(2.0)  # consume terminal line remainder
                except Exception: pass
                self.parser.wait_prompt(3.0)
            self._last_recovery_ok=True
            return True
        except Exception:
            return False

    def download_one(self,event_id:int,initial_header:str|None=None,abort_after_chunks:int=0,
                     expected_item:PhotoListItem|None=None)->dict:
        if event_id<=0: raise ValueError("event ID must be positive")
        pending=0; last_offset=-1; last_data=b""; expected_offset=0; running_crc=0; chunks=0; retries=0
        header=None; tmp_raw=None; raw_stream=None; terminal_seen=False
        retx=0; self._stray=0
        png_path=raw_path=meta_path=None
        try:
            try: header=self._read_header(initial_header)
            except (DeviceCommandError,NoPhotosError):
                terminal_seen=True   # the prompt was consumed; nothing to recover
                raise
            if header.event_id!=event_id: raise ProtocolError(f"requested photo {event_id}, received {header.event_id}")
            if expected_item is not None:
                expected_tuple=(expected_item.width,expected_item.height,expected_item.pixel_format,
                                expected_item.bpp,expected_item.size,expected_item.crc32)
                header_tuple=(header.width,header.height,header.pixel_format,header.bytes_per_pixel,
                              header.total_bytes,header.crc32)
                if header_tuple!=expected_tuple:
                    raise ProtocolError(f"photo {event_id} header differs from photos-list metadata")
            png_path,raw_path,meta_path=self._paths(header)
            self.output.mkdir(parents=True,exist_ok=True)
            tmp_raw=raw_path.with_suffix(raw_path.suffix+".part")
            raw_stream=tmp_raw.open("wb")
            while True:
                line=self._next_photo_record(self.config.photo_timeout)
                fields=line.split(","); tag=fields[0]
                if tag=="IRPHOTO_CHUNK":
                    if len(fields)!=4: raise ProtocolError(f"invalid chunk header: {line}")
                    offset,length,chunk_crc=int(fields[1]),int(fields[2]),int(fields[3],16)
                    if length<=0 or length>header.chunk_size or offset<0 or offset+length>header.total_bytes:
                        raise ProtocolError(f"invalid chunk range: {line}")
                    pending=length
                    try:
                        payload=self.transport.read_exact(length,self.config.photo_timeout,idle_timeout=self.config.photo_timeout)
                    except (ReadTimeout,ConnectionLost) as exc:
                        pending=max(0,length-len(exc.partial))
                        raise
                    pending=0
                    actual=zlib.crc32(payload)&0xffffffff
                    if offset==last_offset and payload==last_data and actual==chunk_crc:
                        retx+=1; self.transport.write(ACK); continue
                    if actual!=chunk_crc or offset!=expected_offset:
                        retries+=1; retx+=1
                        if retries>self.config.retries:
                            self.transport.write(NACK)
                            raise ProtocolError(f"chunk CRC/offset failure at {offset}: expected CRC {chunk_crc:08X}, got {actual:08X}")
                        self.transport.write(NACK); continue
                    raw_stream.write(payload); running_crc=zlib.crc32(payload,running_crc)&0xffffffff
                    last_offset,last_data=offset,payload; expected_offset+=length; chunks+=1; retries=0
                    self.transport.write(ACK)
                    if abort_after_chunks and chunks>=abort_after_chunks:
                        self.transport.write(CANCEL)
                    continue
                if tag=="IRPHOTO_END":
                    if len(fields)!=4: raise ProtocolError(f"invalid photo trailer: {line}")
                    end_id,end_size,end_crc=int(fields[1]),int(fields[2]),int(fields[3],16)
                    raw_stream.flush(); os.fsync(raw_stream.fileno()); raw_stream.close(); raw_stream=None
                    terminal_seen=True
                    try:
                        self.parser.wait_prompt(5.0)
                        self._last_recovery_ok=True
                    except Exception:
                        self._last_recovery_ok=False
                    if (end_id,end_size)!=(header.event_id,header.total_bytes): raise ProtocolError("photo trailer ID/size mismatch")
                    if expected_offset!=header.total_bytes: raise ProtocolError(f"incomplete photo: {expected_offset}/{header.total_bytes}")
                    if running_crc!=header.crc32 or end_crc!=header.crc32 or running_crc!=end_crc:
                        raise ProtocolError(f"frame CRC mismatch: header={header.crc32:08X}, stream={running_crc:08X}, trailer={end_crc:08X}")
                    if abort_after_chunks: raise ProtocolError("abort test requested but device completed the photo")
                    if retx or self._stray:
                        self.log(f"[photo] id={event_id}: UART noise recovered ({retx} chunk retransmission(s), {self._stray} stray byte(s) dropped)")
                    self._finalize(header,tmp_raw,png_path,raw_path,meta_path,running_crc,chunks)
                    return {"event_id":event_id,"status":"complete","chunks":chunks,"crc32":f"{running_crc:08X}","png":str(png_path)}
                if tag=="IRPHOTO_ABORT":
                    terminal_seen=True
                    if raw_stream: raw_stream.close(); raw_stream=None
                    if tmp_raw and tmp_raw.exists(): tmp_raw.unlink()
                    try:
                        self.parser.wait_prompt(3.0)
                        self._last_recovery_ok=True
                    except Exception:
                        self._last_recovery_ok=False
                    return {"event_id":event_id,"status":"aborted","chunks":chunks}
                if tag=="IRPHOTO_ERROR":
                    terminal_seen=True
                    try:
                        self.parser.wait_prompt(3.0)
                        self._last_recovery_ok=True
                    except Exception:
                        self._last_recovery_ok=False
                    raise ProtocolError(f"firmware photo error: {line}")
        except BaseException:
            if raw_stream:
                try: raw_stream.close()
                except Exception: pass
            for temporary in (tmp_raw,
                              png_path.with_suffix(png_path.suffix+".tmp") if png_path else None,
                              meta_path.with_suffix(meta_path.suffix+".tmp") if meta_path else None):
                if temporary and temporary.exists():
                    try: temporary.unlink()
                    except OSError: pass
            if not terminal_seen:
                self._recover(pending)
            raise

    def download_event(self,event_id:int,expected_item:PhotoListItem|None=None)->dict:
        last_error=None
        for attempt in range(self.config.retries+1):
            try:
                self.parser.flush_async()
                self.transport.send_command(f"photo id {event_id}")
                result=self.download_one(event_id,expected_item=expected_item)
                if result.get("status") != "complete":
                    raise ProtocolError(f"photo {event_id} transfer ended with status={result.get('status')}")
                return result
            except ConnectionLost as exc:
                last_error=exc
                if not self._reconnect_and_sync(): break
            except Exception as exc:
                last_error=exc
                if isinstance(exc,(OSError,ValueError,DeviceCommandError,NoPhotosError)) or attempt >= self.config.retries: break
                if not self._last_recovery_ok and not self._reconnect_and_sync(): break
        raise last_error or ProtocolError(f"photo {event_id} failed")

    def _finalize(self,h,tmp_raw,png_path,raw_path,meta_path,crc,chunks):
        from PIL import Image
        preserve_png=False
        preserve_raw=False
        old_meta={}
        if not self.config.overwrite and not self.config.keep_metadata and png_path.exists():
            state=self._png_state(png_path,h.width,h.height,crc)
            if state=="different":
                raise FileExistsError(f"verified output already exists with different content: {png_path} (use 'settings set overwrite true')")
            preserve_png=(state=="match")          # an unreadable/truncated PNG is simply replaced
            preserve_raw=(raw_path.exists() and raw_path.stat().st_size==h.total_bytes and self._file_crc32(raw_path)==crc)
        elif not self.config.overwrite and meta_path.exists():
            try:
                old_meta=json.loads(meta_path.read_text(encoding="utf-8"))
                same_meta=(old_meta.get("crc32")==f"{crc:08X}" and old_meta.get("size_bytes")==h.total_bytes)
            except Exception:
                same_meta=False
            if not same_meta and (png_path.exists() or raw_path.exists()):
                raise FileExistsError(f"verified output already exists with different CRC: {png_path}")
            preserve_png=same_meta and png_path.exists()
            if preserve_png:
                try:
                    from PIL import Image
                    with Image.open(png_path) as existing:
                        existing.verify()
                    with Image.open(png_path) as existing:
                        old_scale=int(old_meta.get("png_scale",1))
                        preserve_png=(existing.size==(h.width*old_scale,h.height*old_scale)
                                      and old_meta.get("png_sha256")==self._file_sha256(png_path))
                except Exception:
                    preserve_png=False
            preserve_raw=(same_meta and raw_path.exists() and raw_path.stat().st_size==h.total_bytes
                          and self._file_crc32(raw_path)==crc)
        data=tmp_raw.read_bytes()
        rgb=self.rgb565_to_rgb(data,self.config.rgb565_byte_order)
        image=Image.frombytes("RGB",(h.width,h.height),rgb)
        if self.config.png_scale>1:
            image=image.resize((h.width*self.config.png_scale,h.height*self.config.png_scale),Image.Resampling.NEAREST)
        tmp_png=png_path.with_suffix(png_path.suffix+".tmp")
        image.save(tmp_png,format="PNG")
        if self.config.overwrite or not preserve_png: os.replace(tmp_png,png_path)
        else: tmp_png.unlink(missing_ok=True)
        if self.config.keep_rgb565:
            if self.config.overwrite or not preserve_raw: os.replace(tmp_raw,raw_path)
            else: tmp_raw.unlink(missing_ok=True)
        else: tmp_raw.unlink(missing_ok=True)
        if not self.config.keep_rgb565:
            raw_path.unlink(missing_ok=True)
        if not self.config.keep_metadata:
            return                      # sidecar JSON is optional
        output_byte_order=old_meta.get("rgb565_byte_order",self.config.rgb565_byte_order) if preserve_png else self.config.rgb565_byte_order
        output_scale=old_meta.get("png_scale",self.config.png_scale) if preserve_png else self.config.png_scale
        meta={"schema_version":1,"event_id":h.event_id,"width":h.width,"height":h.height,
              "pixel_format":"RGB565","bytes_per_pixel":h.bytes_per_pixel,"size_bytes":h.total_bytes,
              "crc32":f"{crc:08X}","chunks":chunks,"rgb565_byte_order":output_byte_order,
              "png_scale":output_scale,"png":png_path.name,"png_sha256":self._file_sha256(png_path),
              "raw":raw_path.name if self.config.keep_rgb565 else None}
        tmp_meta=meta_path.with_suffix(meta_path.suffix+".tmp")
        with tmp_meta.open("w",encoding="utf-8",newline="\n") as f:
            json.dump(meta,f,indent=2);f.write("\n");f.flush();os.fsync(f.fileno())
        os.replace(tmp_meta,meta_path)

    def download_all(self)->int:
        try:
            expected,items,skipped=self.parser.photos_list(timeout=30.0)
        except TransportError:
            if not self._reconnect_and_sync(): raise
            expected,items,skipped=self.parser.photos_list(timeout=30.0)
        self.save_index(expected,items,skipped)
        if not items:
            if expected==0 or skipped>=expected: raise NoPhotosError("device has no downloadable photos")
            raise NoPhotosError("photo list contains no valid downloadable entries")
        success=0; failed=[]
        total=len(items)
        for number,item in enumerate(items,1):
            if self._already_valid(item):
                success+=1
                self.log(f"[photo] {number}/{total} id={item.event_id} already saved, skipped")
                continue
            try:
                result=self.download_event(item.event_id,expected_item=item)
                if result.get("status") != "complete":
                    raise ProtocolError(f"photo {item.event_id} did not complete")
                success += 1
                self.log(f"[photo] {number}/{total} id={item.event_id} saved: {result['png']}")
            except TransportError as exc:
                if success == 0:
                    raise
                failed.append((item.event_id,str(exc)))
                self.log(f"[photo] connection/transport lost at id={item.event_id}: {exc}")
                break
            except Exception as exc:
                failed.append((item.event_id,str(exc)))
                self.log(f"[photo] id={item.event_id} failed after retries: {exc}")
        if failed or skipped:
            details=f"failed IDs: {[x[0] for x in failed]}" if failed else ""
            message=f"downloaded/verified {success}/{len(items)} listed photos; skipped={skipped}; {details}"
            if success > 0 or skipped > 0:
                raise PartialExecutionError(message)
            raise ProtocolError(message)
        self.log(f"[photo] all complete: {success} photos")
        return 0

    def _reconnect_and_sync(self)->bool:
        if not self.config.auto_reconnect: return False
        try:
            self.transport.reconnect(self.config.port,self.config.baud,self.config.reconnect_timeout,self.config.reconnect_interval)
            return self.parser.sync()
        except Exception:
            return False
