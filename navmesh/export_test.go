package navmesh

import "hash/crc32"

// crc32IEEE exposes crc32 for test helpers without exporting it publicly.
func crc32IEEE(b []byte) uint32 { return crc32.ChecksumIEEE(b) }
