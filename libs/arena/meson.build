arena_inc = include_directories('include')

arena_src = files('src' / 'arena.c')

arena_dep = declare_dependency(
    include_directories: arena_inc,
    sources: arena_src,
)

subdir('tests')

tidy_files += [arena_src]
