package navmesh

import (
	"bytes"
	"context"
	"errors"
	"reflect"
	"testing"
)

// 保存-加载往返：数据完全一致，寻路结果一致。
func TestPersistRoundTrip(t *testing.T) {
	obs, zones := testGeometry()
	m1 := NewBuilder(testConfig()).BuildAll(obs, zones)
	m1.Version = 42

	var buf bytes.Buffer
	if err := Save(&buf, m1); err != nil {
		t.Fatal(err)
	}
	m2, err := Load(&buf, testConfig())
	if err != nil {
		t.Fatal(err)
	}
	if m2.Version != 42 {
		t.Fatalf("version lost: %d", m2.Version)
	}
	if !reflect.DeepEqual(m1.Tiles, m2.Tiles) {
		t.Fatal("tiles differ after round trip")
	}

	// 加载后的数据可直接用于寻路，结果与原始一致。
	req := PathRequest{Start: Vec2{2, 2}, Goal: Vec2{62, 62}, Agent: AgentProfile{Radius: 1}}
	a := FindPath(context.Background(), m1, req)
	b := FindPath(context.Background(), m2, req)
	if a.Reason != b.Reason || a.Cost != b.Cost || len(a.Points) != len(b.Points) {
		t.Fatal("path differs between original and loaded mesh")
	}
}

// 损坏的数据必须被校验发现，不得静默使用。
func TestPersistCorruption(t *testing.T) {
	obs, zones := testGeometry()
	m := NewBuilder(testConfig()).BuildAll(obs, zones)
	var buf bytes.Buffer
	if err := Save(&buf, m); err != nil {
		t.Fatal(err)
	}
	data := buf.Bytes()
	// 翻转负载中间的一个字节。
	data[len(data)/2+8] ^= 0xFF
	_, err := Load(bytes.NewReader(data), testConfig())
	if !errors.Is(err, ErrChecksum) {
		t.Fatalf("expected ErrChecksum, got %v", err)
	}
}

// 配置不匹配（构建参数变了）必须视为版本不匹配并触发重建。
func TestPersistConfigMismatch(t *testing.T) {
	m := NewBuilder(testConfig()).BuildAll(nil, nil)
	var buf bytes.Buffer
	if err := Save(&buf, m); err != nil {
		t.Fatal(err)
	}
	cfg := testConfig()
	cfg.CellSize = 2 // 与保存时不一致
	_, err := Load(&buf, cfg)
	if !errors.Is(err, ErrVersionMismatch) {
		t.Fatalf("expected ErrVersionMismatch, got %v", err)
	}
}

// 非法头部必须被拒绝。
func TestPersistBadHeader(t *testing.T) {
	_, err := Load(bytes.NewReader([]byte("garbage-data-not-navmesh")), testConfig())
	if err == nil {
		t.Fatal("expected error for garbage input")
	}
}

// 模拟"重启后校验失败则重建"的完整流程。
func TestReloadOrRebuildFlow(t *testing.T) {
	cfg := testConfig()
	rt := NewRuntime(cfg)
	defer rt.Close()
	obs, zones := testGeometry()
	rt.Rebuild(obs, zones)
	var buf bytes.Buffer
	if err := rt.Save(&buf); err != nil {
		t.Fatal(err)
	}

	// 模拟重启：加载成功则直接使用。
	rt2 := NewRuntime(cfg)
	defer rt2.Close()
	if err := rt2.LoadFrom(bytes.NewReader(buf.Bytes())); err != nil {
		rt2.Rebuild(obs, zones)
	}
	res, err := rt2.FindPath(context.Background(), PathRequest{
		Start: Vec2{2, 2}, Goal: Vec2{62, 62}, Agent: AgentProfile{Radius: 1},
	})
	if err != nil || res.Reason != ReasonNone {
		t.Fatalf("reloaded runtime cannot serve queries: %v %v", res.Reason, err)
	}
}
