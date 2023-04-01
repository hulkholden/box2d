package box2d

import (
	"testing"
)

var result B2AABB

// BenchmarkB2PolygonShape_ComputeAABB-12         18483         64324 ns/op
//
// With hand-rolled min/max
// BenchmarkB2PolygonShape_ComputeAABB-12         66439         18026 ns/op
func BenchmarkB2PolygonShape_ComputeAABB(b *testing.B) {
	polys := make([]B2ShapeInterface, 1000)
	for i := range polys {
		p := NewPolygonShape()
		p.SetAsBox(1, 1)
		polys[i] = p
	}

	tfm := MakeTransform()

	b.ResetTimer()
	for n := 0; n < b.N; n++ {
		for i := range polys {
			aabb := polys[i].ComputeAABB(tfm, 0)
			result.CombineInPlace(aabb)
		}
	}
}
