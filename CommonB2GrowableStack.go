package box2d

type GrowableStack[T any] struct {
	data []T
}

func NewGrowableStack[T any](initialCap int) *GrowableStack[T] {
	return &GrowableStack[T]{
		data: make([]T, 0, initialCap),
	}
}

// Return the stack's length
func (s GrowableStack[T]) GetCount() int {
	return len(s.data)
}

// Push a new element onto the stack
func (s *GrowableStack[T]) Push(value T) {
	s.data = append(s.data, value)
}

// Remove the top element from the stack and return it's value
// If the stack is empty, return zero type.
func (s *GrowableStack[T]) Pop() T {
	if len(s.data) == 0 {
		panic("stack is empty")
	}

	value := s.data[len(s.data)-1]
	s.data = s.data[:len(s.data)-1]
	return value
}
