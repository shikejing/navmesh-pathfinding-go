package navmesh

import (
	"math/rand"
	"testing"
)

func testConfig() Config {
	return Config{CellSize: 1, TileCells: 16, MaxAgentRadius: 4}
}

// uShapedScene returns a 64x64 world with a U-shaped obstacle opening
// upward, plus a cost zone and a forbidden zone.
func uShapedScene() Scene {
	zones := []Zone{
		{ID: "u-left", Kind: ZoneObstacle, Rect: Rect{20, 20, 24, 48}},
		{ID: "u-right", Kind: ZoneObstacle, Rect: Rect{40, 20, 44, 48}},
		{ID: "u-bottom", Kind: ZoneObstacle, Rect: Rect{20, 20, 44, 24}},
		{ID: "mud", Kind: ZoneCost, Rect: Rect{0, 52, 64, 64}, Cost: 3},
		{ID: "keepout", Kind: ZoneCost, Rect: Rect{52, 0, 64, 12}, Forbidden: true},
	}
	return Scene{Bounds: Rect{0, 0, 64, 64}, Zones: zones}
}

func TestBuildDeterministic(t *testing.T) {
	scene := uShapedScene()
	m1 := Build(scene, testConfig())
	m2 := Build(scene, testConfig())
	if m1.Checksum() != m2.Checksum() {
		t.Fatalf("same scene built twice: checksums differ %x != %x", m1.Checksum(), m2.Checksum())
	}
	// Zone order must not matter.
	shuffled := Scene{Bounds: scene.Bounds, Zones: append([]Zone(nil), scene.Zones...)}
	rand.New(rand.NewSource(42)).Shuffle(len(shuffled.Zones), func(i, j int) {
		shuffled.Zones[i], shuffled.Zones[j] = shuffled.Zones[j], shuffled.Zones[i]
	})
	m3 := Build(shuffled, testConfig())
	if m1.Checksum() != m3.Checksum() {
		t.Fatalf("shuffled zone order changed checksum %x != %x", m1.Checksum(), m3.Checksum())
	}
}

func TestConcaveSplitAndAdjacency(t *testing.T) {
	m := Build(uShapedScene(), testConfig())
	if len(m.Regions) < 3 {
		t.Fatalf("expected concave area to split into several regions, got %d", len(m.Regions))
	}
	// Every region's neighbours must list it back (symmetric adjacency).
	for _, r := range m.Regions {
		for _, nb := range r.Neighbors {
			found := false
			for _, back := range m.Regions[m.RegionIndex[nb]].Neighbors {
				if back == r.ID {
					found = true
					break
				}
			}
			if !found {
				t.Fatalf("adjacency not symmetric: %d -> %d but not back", r.ID, nb)
			}
		}
	}
	// The walkable space around the U must be connected: BFS from the
	// region containing a point outside the U must reach the region
	// inside the U's pocket.
	outside, ok := m.regionAt(Vec2{8, 8})
	if !ok || outside < 0 {
		t.Fatal("no region at outside point")
	}
	pocket, ok := m.regionAt(Vec2{32, 30})
	if !ok || pocket < 0 {
		t.Fatal("no region at pocket point")
	}
	seen := map[int64]bool{outside: true}
	queue := []int64{outside}
	for len(queue) > 0 {
		cur := queue[0]
		queue = queue[1:]
		for _, nb := range m.Regions[m.RegionIndex[cur]].Neighbors {
			if !seen[nb] {
				seen[nb] = true
				queue = append(queue, nb)
			}
		}
	}
	if !seen[pocket] {
		t.Fatal("pocket region not reachable from outside region via adjacency")
	}
}

func TestCostAndForbiddenRegions(t *testing.T) {
	m := Build(uShapedScene(), testConfig())
	var mud, keepout *Region
	for i := range m.Regions {
		r := &m.Regions[i]
		c := m.regionCenter(r)
		if (Rect{0, 52, 64, 64}).Contains(c) && r.Cost == 3 {
			mud = r
		}
		if (Rect{52, 0, 64, 12}).Contains(c) && r.Forbidden {
			keepout = r
		}
	}
	if mud == nil {
		t.Fatal("cost zone region not found or cost not applied")
	}
	if keepout == nil {
		t.Fatal("forbidden region not found")
	}
}

func TestClearanceShrinksNearObstacles(t *testing.T) {
	m := Build(uShapedScene(), testConfig())
	near, _ := m.CellAt(m.WorldToCell(Vec2{25.5, 30.5})) // 1.5 from u-left wall (x=24)
	far, _ := m.CellAt(m.WorldToCell(Vec2{8.5, 8.5}))
	if near.Clearance >= far.Clearance {
		t.Fatalf("clearance near obstacle %v should be < far %v", near.Clearance, far.Clearance)
	}
	if near.Clearance > 2 {
		t.Fatalf("clearance near wall too large: %v", near.Clearance)
	}
}
