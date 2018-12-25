
// Fix for: (.text.user_init+0x18): undefined reference to `atexit'
extern "C" int atexit(void (*func)())
{
	return 0;
}
