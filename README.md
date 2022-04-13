# xilinx i2c host lib #

## x_i2c/ xgpio_i2c / emio_i2c 支持 stretching 功能, 可以用于 microblaze

比较下来优先使用 x_i2c > xgpio_i2c ~= emio_i2c

xgpio_i2c 如果开启 stretching, 如果远端异常, 会卡住

## ps_i2c 不支持 stretching 功能