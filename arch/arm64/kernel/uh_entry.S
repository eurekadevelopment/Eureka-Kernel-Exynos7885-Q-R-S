#include <linux/linkage.h>
#include <linux/uh.h>

ENTRY(uh_call)
	hvc	#0
	ret
ENDPROC(uh_call)
