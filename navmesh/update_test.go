package navmesh

import (
	"context"
	"testing"
)

func TestUpdateIsLocal(t *testing.T) {
	m := Build(uShapedScene(), testConfig())
	// Add a small obstacle in the bottom-left corner only.
	nm, diff, err := m.Update([]Zone{
		{ID: "new-rock", Kind: ZoneObstacle, Rect: Rect{4, 56, 8, 60}},
	}, nil)
	if err != nil {
		t.Fatal(err)
	}
	if diff.ToVersion != m.Version+1 {
		t.Fatalf("version did not advance: %+v", diff)
	}
	if len(diff.ChangedTiles) == 0 {
		t.Fatal("expected at least one changed tile")
	}
	// Tile grid is 4x4 (64/16). The rock sits in tile (0,3) -> index 12;
	// with the clearance halo only nearby tiles may change.
	for _, ti := range diff.ChangedTiles {
		tx, ty := ti%m.TilesX, ti/m.TilesX
		if ty < 2 || tx > 1 {
			t.Fatalf("tile %d (%d,%d) changed but is far from the edit", ti, tx, ty)
		}
	}
	// Unchanged tiles must be shared pointers (no full rebuild).
	shared := 0
	for i := range m.Tiles {
		if m.Tiles[i] == nm.Tiles[i] {
			shared++
		}
	}
	if shared == 0 || shared == len(m.Tiles) {
		t.Fatalf("expected partial tile sharing, got %d/%d shared", shared, len(m.Tiles))
	}
	if len(diff.AddedRegions) == 0 && len(diff.RemovedRegions) == 0 {
		t.Fatal("diff should list region changes for changed tiles")
	}
}

func TestUpdateEqualsFullRebuild(t *testing.T) {
	m := Build(uShapedScene(), testConfig())
	upserts := []Zone{
		{ID: "new-rock", Kind: ZoneObstacle, Rect: Rect{4, 56, 8, 60}},
		{ID: "mud", Kind: ZoneCost, Rect: Rect{0, 52, 64, 64}, Cost: 5}, // replace
	}
	nm, _, err := m.Update(upserts, []string{"u-right"})
	if err != nil {
		t.Fatal(err)
	}
	// Build the same resulting scene from scratch.
	full := Build(nm.Scene, testConfig())
	full.Version = nm.Version
	if nm.Checksum() != full.Checksum() {
		t.Fatalf("incremental update diverged from full rebuild: %x != %x",
			nm.Checksum(), full.Checksum())
	}
}

func TestUpdateRemoveZone(t *testing.T) {
	m := Build(uShapedScene(), testConfig())
	nm, diff, err := m.Update(nil, []string{"keepout"})
	if err != nil {
		t.Fatal(err)
	}
	res := nm.FindPath(context.Background(), Query{Start: Vec2{56, 6}, Goal: Vec2{60, 10}})
	if !res.Found {
		t.Fatalf("goal inside removed keepout should be reachable, got %v", res.Reason)
	}
	if len(diff.ChangedTiles) == 0 {
		t.Fatal("expected changed tiles after removing a zone")
	}
	if _, _, err := m.Update(nil, []string{"no-such-zone"}); err == nil {
		t.Fatal("removing unknown zone must error")
	}
}

// TestVersionConsistency: a query on the old snapshot must still see the
// old geometry after an update, and path differences must be explainable
// via the diff.
func TestVersionConsistency(t *testing.T) {
	scene := Scene{
		Bounds: Rect{0, 0, 64, 32},
		Zones: []Zone{
			{ID: "wall-top", Kind: ZoneObstacle, Rect: Rect{30, 0, 34, 12}},
			{ID: "wall-bot", Kind: ZoneObstacle, Rect: Rect{30, 20, 34, 32}},
		},
	}
	m := Build(scene, testConfig())
	q := Query{Start: Vec2{8, 16}, Goal: Vec2{56, 16}}
	before := m.FindPath(context.Background(), q)
	if !before.Found {
		t.Fatal("gate should be open before update")
	}
	// Wall off the gate.
	nm, diff, err := m.Update([]Zone{
		{ID: "gate-shut", Kind: ZoneObstacle, Rect: Rect{30, 12, 34, 20}},
	}, nil)
	if err != nil {
		t.Fatal(err)
	}
	after := nm.FindPath(context.Background(), q)
	if after.Found {
		t.Fatal("gate closed: path must fail on new version")
	}
	// The old snapshot is untouched and still answers with the old path.
	again := m.FindPath(context.Background(), q)
	if !again.Found || again.Version != before.Version {
		t.Fatal("old snapshot changed after update")
	}
	// The diff explains the change: the gate rect intersects changed tiles.
	if len(diff.ChangedTiles) == 0 {
		t.Fatal("diff does not explain the path difference")
	}
	gate := Rect{30, 12, 34, 20}
	explained := false
	for _, ti := range diff.ChangedTiles {
		tx, ty := ti%m.TilesX, ti/m.TilesX
		cs := float64(m.TileCols) * m.Cfg.CellSize
		tr := Rect{float64(tx) * cs, float64(ty) * cs, float64(tx+1) * cs, float64(ty+1) * cs}
		if tr.Intersects(gate) {
			explained = true
		}
	}
	if !explained {
		t.Fatal("no changed tile intersects the edited gate rect")
	}
}
