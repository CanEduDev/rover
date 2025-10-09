wheel_inc = include_directories('include')

wheel_src = files(
    'src' / 'ck.c',
    'src' / 'ck-data.c',
    'src' / 'interrupts.c',
    'src' / 'main.c',
    'src' / 'peripherals.c',
    'src' / 'potentiometer.c',
    'src' / 'report.c',
    'src' / 'wheel-speed.c',
)

wheel_elf = executable(
    'wheel.elf',
    wheel_src,
    include_directories: wheel_inc,
    dependencies: app_deps,
    link_depends: app_linker_script,
    link_args: ['-T@0@'.format(app_linker_script.full_path())],
)

wheel_bin = custom_target(
    output: 'wheel.bin',
    input: wheel_elf,
    command: [objcopy, '-O', 'binary', '-S', '@INPUT@', '@OUTPUT@'],
    build_by_default: true,
)

app_elfs += [wheel_elf]
app_binaries += [wheel_bin]
tidy_files += [wheel_src]
release_files += [wheel_elf, wheel_bin]
