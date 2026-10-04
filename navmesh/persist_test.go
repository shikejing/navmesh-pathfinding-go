package navmesh

import (
	"bytes"
	"context"
	"encoding/binary"
	"testing"
)

func TestSaveLoadRoundtrip(t *testing.T) {
	m := Build(uShapedScene(), testConfig())
	var buf bytes.Buffer
	if err := Save(&buf, m); err != nil {
		t.Fatal(err)
	}
	loaded, err := Load(&buf, testConfig())
	if err != nil {
		t.Fatal(err)
	}
	if loaded.Checksum() != m.Checksum() {
		t.Fatalf("roundtrip checksum mismatch %x != %x", loaded.Checksum(), m.Checksum())
	}
	// Queries on the loaded mesh must match the original.
	q := Query{Start: Vec2{8, 8}, Goal: Vec2{32, 30}, Agent: AgentProfile{Radius: 0.5}}
	r1 := m.FindPath(context.Background(), q)
	r2 := loaded.FindPath(context.Background(), q)
	if r1.Found != r2.Found || r1.Cost != r2.Cost || len(r1.Points) != len(r2.Points) {
		t.Fatal("loaded mesh answers queries differently")
	}
}

func TestLoadDetectsCorruption(t *testing.T) {
	m := Build(uShapedScene(), testConfig())
	var buf bytes.Buffer
	if err := Save(&buf, m); err != nil {
		t.Fatal(err)
	}
	data := buf.Bytes()
	// Flip a byte in the middle of the payload.
	data[len(data)/2] ^= 0xff
	if _, err := Load(bytes.NewReader(data), testConfig()); err == nil {
		t.Fatal("corrupted data must be rejected")
	}
}

func TestLoadDetectsVersionMismatch(t *testing.T) {
	m := Build(uShapedScene(), testConfig())
	var buf bytes.Buffer
	if err := Save(&buf, m); err != nil {
		t.Fatal(err)
	}
	data := buf.Bytes()
	// Bump the format version and fix up the CRC so only the version
	// check can fire.
	binary.LittleEndian.PutUint32(data[4:8], FormatVersion+1)
	fixCRC(data)
	if _, err := Load(bytes.NewReader(data), testConfig()); err == nil {
		t.Fatal("version mismatch must be rejected")
	}
}

func TestLoadDetectsConfigMismatch(t *testing.T) {
	m := Build(uShapedScene(), testConfig())
	var buf bytes.Buffer
	if err := Save(&buf, m); err != nil {
		t.Fatal(err)
	}
	other := testConfig()
	other.CellSize = 2
	if _, err := Load(&buf, other); err == nil {
		t.Fatal("config mismatch must be rejected")
	}
}

func TestLoadRejectsTruncated(t *testing.T) {
	m := Build(uShapedScene(), testConfig())
	var buf bytes.Buffer
	if err := Save(&buf, m); err != nil {
		t.Fatal(err)
	}
	data := buf.Bytes()
	if _, err := Load(bytes.NewReader(data[:len(data)-8]), testConfig()); err == nil {
		t.Fatal("truncated data must be rejected")
	}
	if _, err := Load(bytes.NewReader(data[:4]), testConfig()); err == nil {
		t.Fatal("tiny input must be rejected")
	}
}

// fixCRC recomputes the trailing CRC32 after in-place tampering.
func fixCRC(data []byte) {
	crc := crc32IEEE(data[:len(data)-4])
	binary.LittleEndian.PutUint32(data[len(data)-4:], crc)
}
