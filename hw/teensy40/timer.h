struct timer {
  byte idx;
  IntervalTimer t;
  timer() {};
  timer(unsigned char idx);
  bool init(unsigned char idx);
  void attach_interrupt(void(*fn)(void));
  void set_delay(unsigned int delay, bool periodic);
  void start(void);
  void stop(void);
  void arm(void(*fn)(void), unsigned int delay, bool periodic);
	void (*timer_fn)(void);
	unsigned int delay;
	bool periodic;
};

timer::timer(unsigned char _idx)
{
  init(_idx);
}

bool timer::init(unsigned char _idx)
{
  if (_idx >= 4)
    return false;
  idx = _idx;
  return true;
}

void timer::attach_interrupt(void(*fn)(void))
{
  timer_fn = fn;
  periodic = true;
}

void timer::set_delay(unsigned int delay, bool periodic)
{
  timer::delay = delay;
  timer::periodic = periodic;
}

void timer::start(void)
{
  t.begin(timer_fn, delay);
}

void timer::stop(void)
{
  t.end();
}

void timer::arm(void(*fn)(void), unsigned int delay, bool periodic)
{
  attach_interrupt(fn);
  set_delay(delay, periodic);
  start();
}
