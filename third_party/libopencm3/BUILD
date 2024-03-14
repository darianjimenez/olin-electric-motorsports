cc_library(
    name = "stm32f1xx_hal",
    srcs = glob(["libopencm3/lib/stm32/f1/*.c"]),
    # glob(["libopencm3/lib/stm32/common/*c"], exclude=["libopencm3/lib/stm32/common/lptimer_common_all.c", "libopencm3/lib/stm32/common/dac_common_v2.c", "libopencm3/lib/stm32/common/exti_common_v2.c", "**/*_f47.c"]),
    hdrs = glob(["libopencm3/include/libopencm3/stm32/f1/*.h"]) + 
        glob(["libopencm3/include/libopencm3/stm32/common/*.h"], exclude=["libopencm3/include/libopencm3/stm32/common/dac_common_v2.h"]) + 
        glob(["libopencm3/include/libopencm3/stm32/*.h"], exclude=["libopencm3/include/libopencm3/stm32/lptimer.h"]) + 
        glob(["libopencm3/include/libopencm3/cm3/*.h"]) + 
        glob(["libopencm3/include/libopencm3/usb/*.h"]),
    # include_prefix = "libopencm3",
    strip_include_prefix = "libopencm3/include/",
    target_compatible_with = [
        "//bazel/constraints:stm32f103rbt6",
    ],
    # strip_include_prefix = "include",
    visibility = ["//visibility:public"],
    defines = ["STM32F1"]
)
