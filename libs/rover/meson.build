rover_inc = include_directories('include')

rover_py_def_file = meson.project_source_root() / 'rover_py' / 'rover' / '__init__.py'

rover_assignments_c = custom_target(
    'rover-assignments.c',
    input: files('src' / 'gen-rover-sources.py'),
    output: 'rover-assignments.c',
    command: [python, '@INPUT@', '--source', '@OUTPUT@'],
    depend_files: rover_py_def_file,
)

rover_defs_h = custom_target(
    'rover-defs.h',
    input: files('src' / 'gen-rover-sources.py'),
    output: 'rover-defs.h',
    command: [python, '@INPUT@', '--header', '@OUTPUT@'],
    depend_files: rover_py_def_file,
)

rover_helpers_src = [
    files('src' / 'rover-helpers.c'),
    rover_assignments_c,
    rover_defs_h,
]

rover_src = [files('src' / 'rover.c'), rover_helpers_src]

rover_dep = declare_dependency(
    include_directories: rover_inc,
    sources: rover_src,
)

rover_king_dep = [rover_dep, can_kingdom_king_dep]

subdir('tests')

rover_tidy_files = [rover_src, rover_test_tidy_files]
