package navmesh

import (
	"bytes"
	"reflect"
	"testing"
)

func testConfig() Config {
	return Config{
		WorldMin:  Vec2{0, 0},
		WorldMax:  Vec2{64, 64},
		CellSize:  1,
		TileCells: 16,
	}
}

func testGeometry() ([]Obstacle, []Zone) {
	obs := []Obstacle{
		{ID: 1, Box: AABB{Vec2{10, 10}, Vec2{20, 20}}, MinY: 0, MaxY: 3},
		{ID: 2, Box: AABB{Vec2{40, 5}, Vec2{50, 15}}, MinY: 0, MaxY: 3},
		{ID: 3, Box: AABB{Vec2{0, 32}, Vec2{30, 34}}, MinY: 0, MaxY: 3},  // 墙左段
		{ID: 4, Box: AABB{Vec2{34, 32}, Vec2{64, 34}}, MinY: 0, MaxY: 3}, // 墙右段，中间留 4 格缺口
		{ID: 5, Box: AABB{Vec2{44, 44}, Vec2{52, 52}}, MinY: 0, MaxY: 3},
	}
	zones := []Zone{
		{ID: 1, Box: AABB{Vec2{24, 24}, Vec2{32, 32}}, Cost: 5},
		{ID: 2, Box: AABB{Vec2{56, 0}, Vec2{64, 8}}, Forbidden: true},
	}
	return obs, zones
}

// 同一份几何与配置，构建结果必须完全一致（包括输入顺序打乱后）。
func TestDeterministicBuild(t *testing.T) {
	obs, zones := testGeometry()
	m1 := NewBuilder(testConfig()).BuildAll(obs, zones)
	m2 := NewBuilder(testConfig()).BuildAll(obs, zones)
	if !reflect.DeepEqual(m1.Tiles, m2.Tiles) {
		t.Fatal("same input built twice differs")
	}

	// 输入顺序打乱，结果不变。
	rev := func(o []Obstacle) []Obstacle {
		out := make([]Obstacle, len(o))
		for i, v := range o {
			out[len(o)-1-i] = v
		}
		return out
	}
	m3 := NewBuilder(testConfig()).BuildAll(rev(obs), zones)
	if !reflect.DeepEqual(m1.Tiles, m3.Tiles) {
		t.Fatal("shuffled input order produced different mesh")
	}

	// 序列化字节级一致。
	var b1, b2 bytes.Buffer
	if err := Save(&b1, m1); err != nil {
		t.Fatal(err)
	}
	if err := Save(&b2, m1); err != nil {
		t.Fatal(err)
	}
	if !bytes.Equal(b1.Bytes(), b2.Bytes()) {
		t.Fatal("serialized bytes differ for identical mesh")
	}
}

// 凹区域应被切分为多个凸矩形并建立相邻关系。
func TestConcavePartitionAndAdjacency(t *testing.T) {
	// U 形障碍围出凹区域。
	obs := []Obstacle{
		{ID: 1, Box: AABB{Vec2{8, 8}, Vec2{12, 24}}, MinY: 0, MaxY: 2},
		{ID: 2, Box: AABB{Vec2{20, 8}, Vec2{24, 24}}, MinY: 0, MaxY: 2},
		{ID: 3, Box: AABB{Vec2{8, 8}, Vec2{24, 12}}, MinY: 0, MaxY: 2},
	}
	m := NewBuilder(testConfig()).BuildAll(obs, nil)
	tile := m.Tiles[TileKey{0, 0}]
	if tile == nil {
		t.Fatal("tile missing")
	}
	if len(tile.Regions) < 2 {
		t.Fatalf("concave area should split into multiple regions, got %d", len(tile.Regions))
	}
	// 相邻表对称且索引合法。
	for i, adj := range tile.Adj {
		for _, j := range adj {
			if int(j) >= len(tile.Regions) {
				t.Fatalf("adjacency index out of range: %d", j)
			}
			found := false
			for _, k := range tile.Adj[j] {
				if int(k) == i {
					found = true
				}
			}
			if !found {
				t.Fatalf("adjacency not symmetric: %d -> %d", i, j)
			}
		}
	}
	// 所有区域都是矩形且只覆盖可行走格。
	covered := make(map[[2]int]int)
	for _, r := range tile.Regions {
		for dy := 0; dy < r.H; dy++ {
			for dx := 0; dx < r.W; dx++ {
				key := [2]int{r.X + dx, r.Y + dy}
				covered[key]++
				if !tile.Cells[(r.Y+dy)*tile.W+r.X+dx].walkable() {
					t.Fatalf("region covers unwalkable cell %v", key)
				}
			}
		}
	}
	for k, n := range covered {
		if n != 1 {
			t.Fatalf("cell %v covered %d times", k, n)
		}
	}
}

// 区域代价与禁行标记应正确落到格子上。
func TestZoneCostAndForbidden(t *testing.T) {
	_, zones := testGeometry()
	m := NewBuilder(testConfig()).BuildAll(nil, zones)
	c, _, _, _ := m.cellAt(Vec2{28, 28})
	if c == nil {
		t.Fatal("cell missing")
	}
	if c.Cost != 5 {
		t.Fatalf("expected cost 5, got %d", c.Cost)
	}
	c2, _, _, _ := m.cellAt(Vec2{60, 4})
	if c2.Flags&flagForbidden == 0 {
		t.Fatal("expected forbidden flag")
	}
}
