package navmesh

import (
	"math"
	"time"
)

// FormatVersion is the persistence format version written by Save and
// required by Load.
const FormatVersion uint32 = 1

// Config controls build, query and runtime behaviour. The same scene and
// config must always produce byte-identical meshes.
type Config struct {
	// CellSize is the world-space size of one raster cell. Must be > 0.
	CellSize float64
	// TileCells is the tile edge length in cells. Tiles are the unit of
	// incremental rebuilds. Must be > 0.
	TileCells int
	// QueryWorkers bounds concurrent path queries in a Runtime.
	// <= 0 means a default based on GOMAXPROCS.
	QueryWorkers int
	// QueryTimeout is the default per-query deadline when the caller's
	// context has none. <= 0 means no default timeout.
	QueryTimeout time.Duration
	// MaxExpansions bounds A* node expansions per query; <= 0 means
	// unlimited. Prevents pathological queries from hogging a worker.
	MaxExpansions int
	// MaxAgentRadius is the largest agent radius (world units) for which
	// clearance values are exact. Larger radii are rejected at query time.
	// It also sets the halo used for incremental clearance recomputation.
	// <= 0 means a default of 4 cells.
	MaxAgentRadius float64
}

func (c Config) withDefaults() Config {
	if c.CellSize <= 0 {
		c.CellSize = 1
	}
	if c.TileCells <= 0 {
		c.TileCells = 16
	}
	if c.MaxAgentRadius <= 0 {
		c.MaxAgentRadius = 4 * c.CellSize
	}
	return c
}

// Hash returns a deterministic hash of the config fields that affect the
// built mesh. Used to detect config mismatch on Load.
func (c Config) Hash() uint64 {
	c = c.withDefaults()
	h := fnvOffset
	h = fnvAdd(h, uint64(mathFloatBits(c.CellSize)))
	h = fnvAdd(h, uint64(c.TileCells))
	h = fnvAdd(h, mathFloatBits(c.MaxAgentRadius))
	return h
}

const fnvOffset uint64 = 14695981039346656037
const fnvPrime uint64 = 1099511628211

func fnvAdd(h, v uint64) uint64 {
	for i := 0; i < 8; i++ {
		h ^= (v >> (8 * i)) & 0xff
		h *= fnvPrime
	}
	return h
}

func mathFloatBits(f float64) uint64 { return math.Float64bits(f) }
