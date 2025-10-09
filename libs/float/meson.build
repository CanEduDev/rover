float_inc = include_directories('include')

float_src = files('src' / 'float.c')

float_dep = declare_dependency(
    include_directories: float_inc,
    sources: float_src,
)

subdir('tests')

tidy_files += [float_src]
