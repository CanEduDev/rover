light_array_inc = include_directories('include')

light_array_src = files(
    'src' / 'ck-data.c',
    'src' / 'ck.c',
    'src' / 'lights.c',
    'src' / 'main.c',
    'src' / 'peripherals.c',
)

light_array_elf = executable(
    'light-array.elf',
    light_array_src,
    include_directories: light_array_inc,
    dependencies: app_deps,
    link_depends: app_linker_script,
    link_args: ['-T@0@'.format(app_linker_script.full_path())],
)

light_array_bin = custom_target(
    output: 'light-array.bin',
    input: light_array_elf,
    command: [objcopy, '-O', 'binary', '-S', '@INPUT@', '@OUTPUT@'],
    build_by_default: true,
)

run_target(
    'swd-flash-light-array',
    command: [
        openocd,
        '-f',
        openocd_config,
        '-c',
        'program @0@ verify'.format(bootloader_elf.full_path()),
        '-c',
        'program @0@ verify reset exit'.format(light_array_elf.full_path()),
    ],
    depends: [bootloader_elf, light_array_elf],  # required because full_path doesn't create automatic dependency on these objects
)

release_files += [light_array_elf, light_array_bin]

light_array_tidy_files = [light_array_src]
