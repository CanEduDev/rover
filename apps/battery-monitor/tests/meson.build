battery_test_suite = 'battery'

battery_common_test_src = files(
    '..' / 'src' / 'adc.c',
    '..' / 'src' / 'battery.c',
    '..' / 'src' / 'jumpers.c',
    'battery-fakes.c',
)

battery_adc_test_src = files('test-adc.c')

battery_adc_test = executable(
    'battery-adc-test',
    [battery_adc_test_src, battery_common_test_src],
    include_directories: [battery_monitor_inc, stm32_common_inc],
    dependencies: [math_dep, test_utils_dep, circuits_dep, float_dep],
    native: true,
)

test('battery-adc', battery_adc_test, suite: battery_test_suite)

battery_test_src = files('test-battery.c')

battery_test = executable(
    'battery-test',
    [battery_test_src, battery_common_test_src],
    include_directories: [battery_monitor_inc, stm32_common_inc],
    dependencies: [math_dep, test_utils_dep, circuits_dep, float_dep],
    native: true,
)

test('battery', battery_test, suite: battery_test_suite)

battery_test_tidy_files = [
    battery_common_test_src,
    battery_adc_test_src,
    battery_test_src,
]
