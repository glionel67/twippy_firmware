/**************************************************************************//*****
 * @file     stdio.c
 * @brief    Implementation of newlib syscall
 ********************************************************************************/

#include <stdio.h>
#include <stdarg.h>
#include <sys/types.h>
#include <sys/stat.h>

#include  <errno.h>
#include  <sys/unistd.h> // STDOUT_FILENO, STDERR_FILENO

#include "uart2.h"

#undef errno
extern int errno;
extern int  _end;

__attribute__ ((used))
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

// __attribute__ ((used))
// int link(char *old, char *new) {
// return -1;
// }

// __attribute__ ((used))
// int _close(int file)
// {
//   return -1;
// }

// __attribute__ ((used))
// int _fstat(int file, struct stat *st)
// {
//   st->st_mode = S_IFCHR;
//   return 0;
// }

// __attribute__ ((used))
// int _isatty(int file)
// {
//   return 1;
// }

// __attribute__ ((used))
// int _lseek(int file, int ptr, int dir)
// {
//   return 0;
// }
// __attribute__ ((used))
// int _read(int file, char *ptr, int len)
// {
//   return 0;
// }
__attribute__ ((used))
int _write(int file, char *ptr, int len)
{
   if ((file != STDOUT_FILENO) && (file != STDERR_FILENO))
   {
      errno = EBADF;
      return -1;
   }

   // arbitrary timeout 1000
   int res = uart2_write((uint8_t*)ptr, len);

   // return # of bytes written - as best we can tell
   return (res == OK ? len : 0);
}

__attribute__ ((used))
void abort(void)
{
  /* Abort called */
  while(1);
}
          
/* --------------------------------- End Of File ------------------------------ */
