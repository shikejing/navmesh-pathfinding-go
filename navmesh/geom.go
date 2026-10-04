package navmesh

// Vec2 is a 2D point in world space.
type Vec2 struct {
	X, Y float64
}

// Rect is an axis-aligned rectangle in world space.
type Rect struct {
	MinX, MinY, MaxX, MaxY float64
}

// Contains reports whether p lies inside r (inclusive).
func (r Rect) Contains(p Vec2) bool {
	return p.X >= r.MinX && p.X <= r.MaxX && p.Y >= r.MinY && p.Y <= r.MaxY
}

// Intersects reports whether two rects overlap (inclusive edges).
func (r Rect) Intersects(o Rect) bool {
	return r.MinX <= o.MaxX && o.MinX <= r.MaxX && r.MinY <= o.MaxY && o.MinY <= r.MaxY
}

// ZoneKind classifies a scene zone.
type ZoneKind int

const (
	// ZoneObstacle blocks all agents regardless of size.
	ZoneObstacle ZoneKind = iota
	// ZoneCost is walkable but carries a traversal cost multiplier,
	// an optional forbidden flag and an optional ceiling height.
	ZoneCost
)

// Zone is a rectangular piece of scene geometry. Zones are the unit of
// incremental updates: adding, replacing or removing zones only rebuilds
// the tiles they touch.
type Zone struct {
	ID   string
	Kind ZoneKind
	Rect Rect
	// Cost multiplies traversal cost for ZoneCost zones (>= 1).
	Cost float64
	// Forbidden marks a ZoneCost zone as never traversable.
	Forbidden bool
	// Ceiling is the clearance height of a ZoneCost zone; 0 means unlimited.
	Ceiling float64
}

// Scene is the full input geometry: world bounds plus zones.
type Scene struct {
	Bounds Rect
	Zones  []Zone
}
