package navmesh

import (
	"bytes"
	"encoding/binary"
	"encoding/gob"
	"errors"
	"hash/crc32"
	"io"
	"sort"
)

// 持久化错误。
var (
	ErrChecksum        = errors.New("navmesh: checksum mismatch, data corrupted")
	ErrVersionMismatch = errors.New("navmesh: format or config version mismatch")
	ErrInvalidHeader   = errors.New("navmesh: invalid header")
)

var persistMagic = []byte{'N', 'V', 'M', '1'}

const persistFormatVersion uint32 = 1

// persistHeader 固定头：magic + 格式版本 + 负载长度 + 负载 CRC32。
type persistHeader struct {
	Magic   [4]byte
	Version uint32
	Length  uint32
	CRC     uint32
}

// persistData 为可序列化的导航数据镜像。
// 包含源几何，重启后可在已提交数据上继续增量更新。
type persistData struct {
	Config         Config
	Version        uint64
	CellsX, CellsY int
	Tiles          []persistTile
	Obstacles      []Obstacle
	Zones          []Zone
}

type persistTile struct {
	X, Y    int
	W, H    int
	Cells   []Cell
	Regions []Region
	Adj     [][]int32
}

// configCompatible 校验加载数据的构建配置与当前配置是否一致。
// 不一致的数据属于"版本不匹配"，必须重建，不得静默使用。
func configCompatible(a, b Config) bool {
	a, b = a.withDefaults(), b.withDefaults()
	return a.WorldMin == b.WorldMin && a.WorldMax == b.WorldMax &&
		a.CellSize == b.CellSize && a.TileCells == b.TileCells
}

// Save 将导航数据写入 w，附带格式版本与校验和。
// 不包含源几何；需要重启后继续增量更新时请使用 Runtime.Save。
func Save(w io.Writer, m *NavMesh) error {
	return saveData(w, m, nil, nil)
}

// saveData 序列化网格与（可选的）源几何。
func saveData(w io.Writer, m *NavMesh, obstacles []Obstacle, zones []Zone) error {
	pd := persistData{
		Config:    m.Config,
		Version:   m.Version,
		CellsX:    m.cellsX,
		CellsY:    m.cellsY,
		Tiles:     make([]persistTile, 0, len(m.Tiles)),
		Obstacles: obstacles,
		Zones:     zones,
	}
	// 按确定顺序序列化 tile，保证同一份数据字节级一致。
	keys := make([]TileKey, 0, len(m.Tiles))
	for k := range m.Tiles {
		keys = append(keys, k)
	}
	sort.Slice(keys, func(i, j int) bool {
		if keys[i].Y != keys[j].Y {
			return keys[i].Y < keys[j].Y
		}
		return keys[i].X < keys[j].X
	})
	for _, k := range keys {
		t := m.Tiles[k]
		pd.Tiles = append(pd.Tiles, persistTile{
			X: t.X, Y: t.Y, W: t.W, H: t.H,
			Cells: t.Cells, Regions: t.Regions, Adj: t.Adj,
		})
	}
	var buf bytes.Buffer
	if err := gob.NewEncoder(&buf).Encode(pd); err != nil {
		return err
	}
	payload := buf.Bytes()
	h := persistHeader{
		Version: persistFormatVersion,
		Length:  uint32(len(payload)),
		CRC:     crc32.ChecksumIEEE(payload),
	}
	copy(h.Magic[:], persistMagic)
	if err := binary.Write(w, binary.LittleEndian, &h); err != nil {
		return err
	}
	_, err := w.Write(payload)
	return err
}

// Load 读取并校验导航数据；校验和或版本不匹配时返回错误，
// 调用方应据错误触发重建，绝不静默使用过期/损坏数据。
func Load(r io.Reader, cfg Config) (*NavMesh, error) {
	m, _, _, err := loadData(r, cfg)
	return m, err
}

// loadData 读取并校验，返回网格与（可能为空的）源几何。
func loadData(r io.Reader, cfg Config) (*NavMesh, []Obstacle, []Zone, error) {
	var h persistHeader
	if err := binary.Read(r, binary.LittleEndian, &h); err != nil {
		return nil, nil, nil, err
	}
	if h.Magic != [4]byte(persistMagic) {
		return nil, nil, nil, ErrInvalidHeader
	}
	if h.Version != persistFormatVersion {
		return nil, nil, nil, ErrVersionMismatch
	}
	payload := make([]byte, h.Length)
	if _, err := io.ReadFull(r, payload); err != nil {
		return nil, nil, nil, err
	}
	if crc32.ChecksumIEEE(payload) != h.CRC {
		return nil, nil, nil, ErrChecksum
	}
	var pd persistData
	if err := gob.NewDecoder(bytes.NewReader(payload)).Decode(&pd); err != nil {
		return nil, nil, nil, err
	}
	if !configCompatible(pd.Config, cfg) {
		return nil, nil, nil, ErrVersionMismatch
	}
	m := &NavMesh{
		Version: pd.Version,
		Config:  pd.Config.withDefaults(),
		Tiles:   make(map[TileKey]*Tile, len(pd.Tiles)),
		cellsX:  pd.CellsX,
		cellsY:  pd.CellsY,
	}
	for _, pt := range pd.Tiles {
		m.Tiles[TileKey{pt.X, pt.Y}] = &Tile{
			X: pt.X, Y: pt.Y, W: pt.W, H: pt.H,
			Cells: pt.Cells, Regions: pt.Regions, Adj: pt.Adj,
		}
	}
	return m, pd.Obstacles, pd.Zones, nil
}
