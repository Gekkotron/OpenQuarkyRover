# Host-runnable unit tests. No ESP-IDF, no target, no serial cable —
# each test compiles the module under test with mocked I/O and asserts
# on the captured effects.
#
# Usage:
#   make            # build every test then run them
#   make clean      # remove binaries + vendored Unity
#
# Unity is fetched on first build (tag v2.5.2 pinned to keep the source
# reproducible). If offline, pre-populate unity/ with the three files
# manually before `make`.

CC       := cc
CFLAGS   := -std=c11 -Wall -Wextra -g -O0 -I. -I../../main -Iunity
LDFLAGS  :=

UNITY_SRC := unity/unity.c
TESTS     := es8311_init_test command_bus_test led_indicator_test

all: $(TESTS) run

unity/unity.c unity/unity.h unity/unity_internals.h:
	@mkdir -p unity
	@echo "== fetching Unity v2.5.2 =="
	@curl -fsSL https://raw.githubusercontent.com/ThrowTheSwitch/Unity/v2.5.2/src/unity.c > unity/unity.c
	@curl -fsSL https://raw.githubusercontent.com/ThrowTheSwitch/Unity/v2.5.2/src/unity.h > unity/unity.h
	@curl -fsSL https://raw.githubusercontent.com/ThrowTheSwitch/Unity/v2.5.2/src/unity_internals.h > unity/unity_internals.h

es8311_init_test: es8311_init_test.c $(UNITY_SRC)
	$(CC) $(CFLAGS) $^ -o $@ $(LDFLAGS)

command_bus_test: command_bus_test.c $(UNITY_SRC)
	$(CC) $(CFLAGS) $^ -o $@ $(LDFLAGS)

led_indicator_test: led_indicator_test.c $(UNITY_SRC)
	$(CC) $(CFLAGS) $^ -o $@ $(LDFLAGS)

run: $(TESTS)
	@set -e; for t in $(TESTS); do echo "== $$t =="; ./$$t; done

clean:
	rm -f $(TESTS)
	rm -rf unity/

.PHONY: all run clean
