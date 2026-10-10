zephyr_library()
zephyr_library_sources(word_memcpy.c)
# the byte loops must stay loops, not become calls to memcpy
zephyr_library_compile_options(-fno-tree-loop-distribute-patterns -fno-builtin)
