vesc_control_inc = include_directories('include')

vesc_control_src = files('src' / 'main.c', 'src' / 'peripherals.c')

vesc_control_elf = executable(
    'vesc-control.elf',
    vesc_control_src,
    include_directories: vesc_control_inc,
    dependencies: app_deps,
    link_depends: app_linker_script,
    link_args: ['-T@0@'.format(app_linker_script.full_path())],
)

vesc_control_bin = custom_target(
    output: 'vesc-control.bin',
    input: vesc_control_elf,
    command: [objcopy, '-O', 'binary', '-S', '@INPUT@', '@OUTPUT@'],
    build_by_default: true,
)

run_target(
    'swd-flash-vesc-control',
    command: [
        openocd,
        '-f',
        openocd_config,
        '-c',
        'program @0@ verify'.format(bootloader_elf.full_path()),
        '-c',
        'program @0@ verify reset exit'.format(vesc_control_elf.full_path()),
    ],
    depends: [bootloader_elf, vesc_control_elf],  # required because full_path doesn't create automatic dependency on these objects
)

release_files += [vesc_control_elf, vesc_control_bin]

vesc_control_tidy_files = [vesc_control_src]
