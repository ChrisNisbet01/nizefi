/**************************************************************************//*****
 * @file     stdio.c
 * @brief    Implementation of newlib syscall
 ********************************************************************************/

#include <stdio.h>
#include <stddef.h>
#include <stdarg.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

#undef errno
extern int errno;
extern int  _end;

caddr_t _sbrk ( int incr )
{
  static unsigned char *heap = NULL;
  unsigned char *prev_heap;

  if (heap == NULL) {
    heap = (unsigned char *)&_end;
  }
  prev_heap = heap;

  heap += incr;

  return (caddr_t) prev_heap;
}

int link(const char *old, const char *new) 
{
    return -1;
}

int _close(int file)
{
  return -1;
}

int _fstat(int file, struct stat *st)
{
  st->st_mode = S_IFCHR;
  return 0;
}

int _isatty(int file)
{
  return 1;
}

int _lseek(int file, int ptr, int dir)
{
  return 0;
}

int _read(int file, char *ptr, int len)
{
  return 0;
}

int _write(int file, char *ptr, int len)
{
    int debug_put_block(void * data, size_t len);

    if (file == STDOUT_FILENO || file == STDERR_FILENO)
    {
        len = debug_put_block(ptr, len);
    }

    return len;
}

void abort(void)
{
  /* Abort called */
  while(1);
}
          
/* --------------------------------- End Of File ------------------------------ */
