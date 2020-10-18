MAIN?=main
APP?=serial
ADD?=gps.c PREDICT.c

LDFLAGS?= \
        -mpatmos-method-cache-size=0x1000 \
        -mpatmos-stack-base=0x080000 -mpatmos-shadow-stack-base=0x078000 \
        -Xgold --defsym -Xgold __heap_end=0x070000

all:
	patmos-clang -I ../.. -O2 $(LDFLAGS) $(ADD) $(MAIN).c -o $(APP).elf -lm
	patmos-clang -I ../.. -O2 $(LDFLAGS) $(ADD) $(MAIN).c -o ~/t-crest/patmos/tmp/$(APP).elf -lm
