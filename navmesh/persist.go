package navmesh

import (
	"bytes"
	"encoding/binary"
	"encoding/gob"
	"fmt"
	"hash/crc32"
	"io"
)

var persistMagic = []byte("NVM1")

// Save serialises the mesh with a format version and checksum so that
// corruption and version mismatches are detected on Load.
func Save(w io.Writer, m *NavMesh) error {
	var payload bytes.Buffer
	if err := gob.NewEncoder(&payload).Encode(m); err != nil {
		return fmt.Errorf("navmesh: encode: %w", err)
	}
	var buf bytes.Buffer
	buf.Write(persistMagic)
	hdr := make([]byte, 8*3)
	binary.LittleEndian.PutUint32(hdr[0:4], FormatVersion)
	binary.LittleEndian.PutUint64(hdr[8:16], m.Cfg.Hash())
	binary.LittleEndian.PutUint64(hdr[16:24], m.Checksum())
	buf.Write(hdr)
	buf.Write(payload.Bytes())
	crc := crc32.ChecksumIEEE(buf.Bytes())
	tail := make([]byte, 4)
	binary.LittleEndian.PutUint32(tail, crc)
	buf.Write(tail)
	_, err := w.Write(buf.Bytes())
	return err
}

// Load reads a mesh written by Save. It returns an error when the data is
// corrupt, the format version differs, or the config hash does not match;
// callers must rebuild in those cases. Stale data is never returned
// silently.
func Load(r io.Reader, cfg Config) (*NavMesh, error) {
	data, err := io.ReadAll(r)
	if err != nil {
		return nil, fmt.Errorf("navmesh: read: %w", err)
	}
	if len(data) < 4+24+4 { // magic + header + crc
		return nil, fmt.Errorf("navmesh: data too short (%d bytes)", len(data))
	}
	body, tail := data[:len(data)-4], data[len(data)-4:]
	wantCRC := binary.LittleEndian.Uint32(tail)
	if got := crc32.ChecksumIEEE(body); got != wantCRC {
		return nil, fmt.Errorf("navmesh: checksum mismatch (corrupt data)")
	}
	if !bytes.Equal(body[:4], persistMagic) {
		return nil, fmt.Errorf("navmesh: bad magic")
	}
	if v := binary.LittleEndian.Uint32(body[4:8]); v != FormatVersion {
		return nil, fmt.Errorf("navmesh: format version %d, want %d (rebuild required)", v, FormatVersion)
	}
	cfgHash := binary.LittleEndian.Uint64(body[12:20])
	if cfgHash != cfg.withDefaults().Hash() {
		return nil, fmt.Errorf("navmesh: config hash mismatch (rebuild required)")
	}
	storedSum := binary.LittleEndian.Uint64(body[20:28])
	var m NavMesh
	if err := gob.NewDecoder(bytes.NewReader(body[28:])).Decode(&m); err != nil {
		return nil, fmt.Errorf("navmesh: decode: %w", err)
	}
	if got := m.Checksum(); got != storedSum {
		return nil, fmt.Errorf("navmesh: mesh checksum mismatch after decode (corrupt data)")
	}
	return &m, nil
}
