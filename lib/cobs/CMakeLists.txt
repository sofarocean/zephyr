zephyr_include_directories_ifdef(
  CONFIG_COBS
  .
)

zephyr_sources_ifdef(
  CONFIG_COBS
  cobs.c
)
