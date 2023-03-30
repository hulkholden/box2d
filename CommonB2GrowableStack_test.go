package box2d_test

import (
	"testing"

	"github.com/hulkholden/box2d"
)

func TestB2GrowableStack(t *testing.T) {
	s := box2d.NewB2GrowableStack[int](4)
	if got := s.GetCount(); got != 0 {
		t.Errorf("GetCount() = %d, want 0", got)
	}

	s.Push(1)
	if got, want := s.GetCount(), 1; got != want {
		t.Errorf("GetCount() = %d, want %d", got, want)
	}

	s.Push(2)
	if got, want := s.GetCount(), 2; got != want {
		t.Errorf("GetCount() = %d, want %d", got, want)
	}

	s.Push(3)
	if got, want := s.GetCount(), 3; got != want {
		t.Errorf("GetCount() = %d, want %d", got, want)
	}

	if got, want := s.Pop(), 3; got != want {
		t.Errorf("Pop() = %d, want %d", got, want)
	}
	if got, want := s.GetCount(), 2; got != want {
		t.Errorf("GetCount() = %d, want %d", got, want)
	}

	if got, want := s.Pop(), 2; got != want {
		t.Errorf("Pop() = %d, want %d", got, want)
	}
	if got, want := s.GetCount(), 1; got != want {
		t.Errorf("GetCount() = %d, want %d", got, want)
	}

	if got, want := s.Pop(), 1; got != want {
		t.Errorf("Pop() = %d, want %d", got, want)
	}
	if got, want := s.GetCount(), 0; got != want {
		t.Errorf("GetCount() = %d, want %d", got, want)
	}
}
