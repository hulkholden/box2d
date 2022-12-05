package box2d

type B2GrowableStack[T any] struct {
	data []T
}

func NewB2GrowableStack[T any](initialCap int) *B2GrowableStack[T] {
	return &B2GrowableStack[T]{
		data: make([]T, 0, initialCap),
	}
}

// Return the stack's length
func (s B2GrowableStack[T]) GetCount() int {
	return len(s.data)
}

// Push a new element onto the stack
func (s *B2GrowableStack[T]) Push(value T) {
	s.data = append(s.data, value)
}

// Remove the top element from the stack and return it's value
// If the stack is empty, return zero type.
func (s *B2GrowableStack[T]) Pop() T {
	if len(s.data) == 0 {
		panic("stack is empty")
	}

	value := s.data[len(s.data)-1]
	s.data = s.data[:len(s.data)-1]
	return value
}
