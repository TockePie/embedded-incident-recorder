# Хостові тести:
CC      := gcc
BUILD   := build
CFLAGS  := -std=gnu99 -Wall -Wextra -g -O0 -I. -I../firmware/App/Inc

.PHONY: run clean coverage
run: | $(BUILD)
	@echo "No host tests specified yet."

# Покриття: 
coverage: | $(BUILD)
	@echo "Code coverage report skipped (no active tests)."

$(BUILD):
	mkdir -p $@

clean:
	rm -rf $(BUILD)
