battery_monitor_inc = include_directories('include')

battery_monitor_src = files(
    'src' / 'adc.c',
    'src' / 'alarm.c',
    'src' / 'battery.c',
    'src' / 'ck-data.c',
    'src' / 'ck-rx-letters.c',
    'src' / 'freertos-tasks.c',
    'src' / 'interrupts.c',
    'src' / 'jumpers.c',
    'src' / 'led.c',
    'src' / 'main.c',
    'src' / 'peripherals.c',
    'src' / 'potentiometer.c',
    'src' / 'power.c',
)

battery_monitor_elf = executable(
    'battery-monitor.elf',
    battery_monitor_src,
    include_directories: battery_monitor_inc,
    dependencies: app_deps,
    link_depends: app_linker_script,
    link_args: ['-T@0@'.format(app_linker_script.full_path())],
)

battery_monitor_bin = custom_target(
    output: 'battery-monitor.bin',
    input: battery_monitor_elf,
    command: [objcopy, '-O', 'binary', '-S', '@INPUT@', '@OUTPUT@'],
    build_by_default: true,
)

subdir('tests')

app_elfs += [battery_monitor_elf]
app_binaries += [battery_monitor_bin]
tidy_files += [battery_monitor_src]
release_files += [battery_monitor_elf, battery_monitor_bin]
