package box2d_test

import (
	"testing"

	"github.com/ByteArena/box2d"
	"github.com/google/go-cmp/cmp"
	"github.com/google/go-cmp/cmp/cmpopts"
)

var closeEnough = cmpopts.EquateApprox(0, 1e-5)

func TestB2Vec(t *testing.T) {
	v := box2d.MakeB2Vec2(1, 2)
	if got, want := v, (box2d.B2Vec2{1, 2}); !cmp.Equal(got, want, closeEnough) {
		t.Errorf("MakeB2Vec2() = %v; want %v", got, want)
	}

	v.SetZero()
	if got, want := v, box2d.MakeB2Vec2(0, 0); !cmp.Equal(got, want, closeEnough) {
		t.Errorf("SetZero() = %v; want %v", got, want)
	}

	v.Set(3, 4)
	if got, want := v, box2d.MakeB2Vec2(3, 4); !cmp.Equal(got, want, closeEnough) {
		t.Errorf("Set() = %v; want %v", got, want)
	}

	if got, want := v.OperatorNegate(), box2d.MakeB2Vec2(-3, -4); !cmp.Equal(got, want, closeEnough) {
		t.Errorf("OperatorNegate() = %v; want %v", got, want)
	}

	if got, want := v.OperatorIndexGet(0), 3.; !cmp.Equal(got, want, closeEnough) {
		t.Errorf("OperatorIndexGet(0) = %f; want %f", got, want)
	}
	if got, want := v.OperatorIndexGet(1), 4.; !cmp.Equal(got, want, closeEnough) {
		t.Errorf("OperatorIndexGet(1) = %f; want %f", got, want)
	}

	v.OperatorIndexSet(0, 7)
	if got, want := v, box2d.MakeB2Vec2(7, 4); !cmp.Equal(got, want, closeEnough) {
		t.Errorf("OperatorIndexSet(0, _) = %v; want %v", got, want)
	}
	v.OperatorIndexSet(1, 9)
	if got, want := v, box2d.MakeB2Vec2(7, 9); !cmp.Equal(got, want, closeEnough) {
		t.Errorf("OperatorIndexSet(1, _) = %v; want %v", got, want)
	}

	v.OperatorPlusInplace(box2d.MakeB2Vec2(1, 2))
	if got, want := v, box2d.MakeB2Vec2(8, 11); !cmp.Equal(got, want, closeEnough) {
		t.Errorf("OperatorPlusInplace() = %v; want %v", got, want)
	}

	v.OperatorMinusInplace(box2d.MakeB2Vec2(4, 1))
	if got, want := v, box2d.MakeB2Vec2(4, 10); !cmp.Equal(got, want, closeEnough) {
		t.Errorf("OperatorMinusInplace() = %v; want %v", got, want)
	}

	v.OperatorScalarMulInplace(2)
	if got, want := v, box2d.MakeB2Vec2(8, 20); !cmp.Equal(got, want, closeEnough) {
		t.Errorf("OperatorScalarMulInplace() = %v; want %v", got, want)
	}

	if got, want := v.Length(), 21.540659; !cmp.Equal(got, want, closeEnough) {
		t.Errorf("Length() = %f; want %f", got, want)
	}
	if got, want := v.LengthSquared(), 464.; !cmp.Equal(got, want, closeEnough) {
		t.Errorf("LengthSquared() = %f; want %f", got, want)
	}

	if got, want := v.Normalize(), 21.540659; !cmp.Equal(got, want, closeEnough) {
		t.Errorf("Normalize() = %f; want %f", got, want)
	}
	if got, want := v, box2d.MakeB2Vec2(0.371391, 0.928477); !cmp.Equal(got, want, closeEnough) {
		t.Errorf("v after Normalize() = %f; want %f", got, want)
	}
}

func TestB2SweepGetTransform(t *testing.T) {
	tests := map[string]struct {
		sweep box2d.B2Sweep
		beta  float64
		want  box2d.B2Transform
	}{
		"at 0.0": {
			sweep: box2d.B2Sweep{
				C0:     box2d.MakeB2Vec2(-2.0, 4.0),
				C:      box2d.MakeB2Vec2(3.0, 8.0),
				A0:     0.5,
				A:      5.0,
				Alpha0: 0.0,
			},
			beta: 0.0,
			want: box2d.B2Transform{
				P: box2d.MakeB2Vec2(-2.0, 4.0),
				Q: box2d.MakeB2RotFromAngle(0.5),
			},
		},
		"at 1.0": {
			sweep: box2d.B2Sweep{
				C0:     box2d.MakeB2Vec2(-2.0, 4.0),
				C:      box2d.MakeB2Vec2(3.0, 8.0),
				A0:     0.5,
				A:      5.0,
				Alpha0: 0.0,
			},
			beta: 1.0,
			want: box2d.B2Transform{
				P: box2d.MakeB2Vec2(3.0, 8.0),
				Q: box2d.MakeB2RotFromAngle(5.0),
			},
		},
	}

	for tn, tc := range tests {
		t.Run(tn, func(t *testing.T) {
			var got box2d.B2Transform
			tc.sweep.GetTransform(&got, tc.beta)
			if !cmp.Equal(got, tc.want, closeEnough) {
				t.Errorf("GetTransform() = %+v; want %+v", got, tc.want)
			}
		})
	}
}
