package box2d

import "time"

// Timer for profiling. This has platform specific code and may
// not work on every platform.
type Timer struct {
	m_start time.Time
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
// B2Timer.cpp
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

func MakeTimer() Timer {
	timer := Timer{}
	timer.Reset()
	return timer
}

func (timer *Timer) Reset() {
	timer.m_start = time.Now()
}

func (timer Timer) GetMilliseconds() float64 {
	return time.Since(timer.m_start).Seconds() * 1000
}
