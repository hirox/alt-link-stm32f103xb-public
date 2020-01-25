set print pretty on
set remotetimeout 5
file build/stm32f1.elf
target remote localhost:3333
