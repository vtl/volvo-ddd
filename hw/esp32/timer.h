struct timer {
  hw_timer_t *t = NULL;
  timer() {};
  timer(unsigned char idx);
  bool init(unsigned char idx);
  void attach_interrupt(void(*fn)(void));
  void set_delay(unsigned int delay, bool periodic);
  void start(void);
  void stop(void);
  void arm(void(*fn)(void), unsigned int delay, bool periodic);
};

timer::timer(unsigned char idx)
{
  init(idx);
}

bool timer::init(unsigned char idx)
{
  if (idx >= 4)
    return false;
  t = timerBegin(idx, 80, true);
  return true;
}

void timer::attach_interrupt(void(*fn)(void))
{
  timerAttachInterrupt(t, fn, true);
}

void timer::set_delay(unsigned int delay, bool periodic)
{
  timerAlarmWrite(t, delay, periodic);
}

void timer::start(void)
{
  timerAlarmEnable(t);
}

void timer::stop(void)
{
  timerEnd(t);
}

void timer::arm(void(*fn)(void), unsigned int delay, bool periodic)
{
  attach_interrupt(fn);
  set_delay(delay, periodic);
  start();
}
