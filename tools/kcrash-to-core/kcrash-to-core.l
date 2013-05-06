/* Copyright (c) 2013 The Chromium OS Authors. All rights reserved.
 * Use of this source code is governed by the GNU General Public License v2.
 *
 * Reads a kernel log with a panic and produces a core file with that stack.
 *
 * TODO(semenzato):
 *
 * - Architecture needs to be abstracted out, need to add at least 386 and ARM.
 * - Discover architecture automatically with an additional pass on the log.
 */

%{

#include <elf.h>
#include <fcntl.h>
#include <getopt.h>
#include <inttypes.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdlib.h>
#include <sys/procfs.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>

#define STACK_SIZE 8192  /* XXX this won't be true forever */
#define STACK_SIZE_64 (STACK_SIZE >> 3)  /* size in 64-bit words */

/* Declarations used in rule actions. */
uint64_t rsp, rbp, rip;
uint64_t kernel_offset, kernel_start;
uint64_t kernel_reloc_low, kernel_reloc_high;
uint64_t stack[STACK_SIZE_64];
int stack_index;
bool kaslr_fix_addresses_flag = false;
bool found_kernel_offset = false;

static void fatal(const char *format, ...);
void check_for_empty_stack(void);
uint64_t hex_to_u64(const char *text);


%}

D       [0-9]
XD      [0-9a-f]
TSTAMP  \[\ *{D}+\.{D}+\]
PREFIX  (<{D}+>)?{TSTAMP}

%x STACK RSP RBP RIP
%x KOFFSET KSTART KRELOCLOW KRELOCHIGH

%%

{PREFIX}" RIP: "{XD}{4}":[<"   BEGIN(RIP);
<RIP>{XD}+		       rip = hex_to_u64(yytext); BEGIN(0);

{PREFIX}" RSP: "{XD}{4}":"     BEGIN(RSP);
<RSP>{XD}+		       rsp = hex_to_u64(yytext); BEGIN(0);

{PREFIX}" RBP: "	       BEGIN(RBP);
<RBP>{XD}+		       rbp = hex_to_u64(yytext); BEGIN(0);

{PREFIX}" Stack:"	       BEGIN(STACK); check_for_empty_stack();
<STACK>{PREFIX}" "	       /* skip */
<STACK>\n		       /* skip */
<STACK>" "{XD}+		       stack[stack_index++] = hex_to_u64(yytext + 1);
<STACK>{PREFIX}" Call Trace:"  BEGIN(0);

{PREFIX}" Kernel Offset: "     BEGIN(KOFFSET);
<KOFFSET>"0x"{XD}+	       { kernel_offset = hex_to_u64(yytext + 2);
				 found_kernel_offset = true; }
<KOFFSET>\n		       fatal("\"Kernel Offset\" line without range\n");
<KOFFSET>" from "	       BEGIN(KSTART);
<KSTART>"0x"{XD}+	       kernel_start = hex_to_u64(yytext + 2);
<KSTART>" (relocation range: " BEGIN(KRELOCLOW);
<KRELOCLOW>0x{XD}+	       kernel_reloc_low = hex_to_u64(yytext + 2);
<KRELOCLOW>"-"		       BEGIN(KRELOCHIGH);
<KRELOCHIGH>"0x"{XD}+	       kernel_reloc_high = hex_to_u64(yytext + 2);
<KRELOCHIGH>")"		       BEGIN(0);

 /* Ignore the rest of the input, unless we're in the middle of parsing
  * something */
<INITIAL>.|\n		      /* ignore */
<*>.|\n			      fatal("unexpected input in state %d\n", YY_START);

%%
resetline (workaround for broken line numbering)

const char *program_name;
struct user_regs_struct regs;
struct user_fpregs_struct fpregs;

static void fatal(const char *format, ...)
{
	va_list ap;
	va_start(ap, format);
	fprintf(stderr, "%s: ", program_name);
	vfprintf(stderr, format, ap);
	exit(1);
}

uint64_t hex_to_u64(const char *text)
{
	char *endptr;
	uint64_t value = strtoul(text, &endptr, 16);
	if (*endptr != '\0')
		/* "cannot happen" */
		fatal("internal scanner error: text: %s\n", text);
	return value;
}

void check_for_empty_stack(void)
{
	if (stack_index != 0)
		fatal("multiple crash dumps found\n");
}

static inline void kaslr_adjust(uint64_t *paddr)
{
	if (kernel_start <= *paddr &&
	    *paddr <= kernel_reloc_high)
		*paddr -= kernel_offset;
}

/* Adjusts stack and register values that look like kaslr-relocated addresses
 * by the amount of kernel offset.  This is by no means safe, but it may be
 * good enough.
 */
void kaslr_fix_addresses(void)
{
	int i;
	if (!found_kernel_offset)
		fatal("unknown kernel offset: cannot apply kaslr fix\n");

	kaslr_adjust(&rip);
	for (i = 0; i < stack_index; i++) {
		kaslr_adjust(&stack[i]);
	}
}

void output(int fd, const void *obj, int size, int offset)
{
	int written;

	if (lseek(fd, offset, SEEK_SET) < 0)
		fatal("lseek: %s\n", strerror(errno));
	while (size > 0) {
		written = write(fd, obj, size);
		if (written < 0 && errno != EINTR)
			fatal("write: %s\n", strerror, errno);
		size -= written;
		obj += written;
	}
}

/* Fills in the missing parts, performs consistency checks, and outputs all
 * pieces of the core file. |fd| is the file descriptor for the output file,
 * which must be a seekable file. |elf_header| is the core file
 * header. |program_headers| is an array containing |program_headers_count|
 * headers.  The rest of the arguments are pointers to uninterpreted data for
 * each program header.
 */
void elf_output(int fd, Elf64_Ehdr *elf_header, Elf64_Phdr *program_headers,
		int program_headers_count, ...)
{
	va_list ap;
	va_start(ap, program_headers_count);
	int i;
	int size, offset;

	elf_header->e_ident[EI_MAG0]    = ELFMAG0;
	elf_header->e_ident[EI_MAG1]    = ELFMAG1;
	elf_header->e_ident[EI_MAG2]    = ELFMAG2;
	elf_header->e_ident[EI_MAG3]    = ELFMAG3;
	elf_header->e_ident[EI_CLASS]   = ELFCLASS64;
	elf_header->e_ident[EI_DATA]    = ELFDATA2LSB;
	elf_header->e_ident[EI_VERSION] = EV_CURRENT;

	elf_header->e_machine = EM_X86_64;  // or EM_386, or EM_ARM.

	elf_header->e_type = ET_CORE;
	elf_header->e_phoff = sizeof(*elf_header);
	elf_header->e_ehsize = sizeof(*elf_header);
	elf_header->e_phentsize = sizeof(*program_headers);
	elf_header->e_phnum = program_headers_count;

	output(fd, elf_header, sizeof(*elf_header), 0);

	/* Compute offsets of data sections for each program header. */
	offset = sizeof(*elf_header) +
		sizeof(*program_headers) * program_headers_count;

	for (i = 0; i < program_headers_count; i++) {
		int align = program_headers[i].p_align;
		if (align > 0) {
			offset += align - (offset % align);
		}
		program_headers[i].p_offset = offset;
		offset += program_headers[i].p_filesz;
	}

	/* Output headers and data. */
	output(fd, program_headers,
	       program_headers_count * sizeof(*program_headers),
	       elf_header->e_phoff);

	for (i = 0; i < program_headers_count; i++) {
		char *p = va_arg(ap, char *);
		size = program_headers[i].p_filesz;
		offset = program_headers[i].p_offset;
		output(fd, p, size, offset);
	}
}

void *add_note(void *buffer, int type, void *record, int record_size)
{
	Elf64_Nhdr *notes_header = (Elf64_Nhdr *) buffer;
	static const char core_string[8] = "CORE\0\0\0\0";
	notes_header->n_namesz = 5;
	notes_header->n_descsz = record_size;
	notes_header->n_type = type;
	memcpy(buffer, notes_header, sizeof(Elf64_Nhdr));
	buffer += sizeof(Elf64_Nhdr);
	memcpy(buffer, core_string, sizeof(core_string));
	buffer += sizeof(core_string);
	memcpy(buffer, record, record_size);
	buffer += record_size;
	return buffer;
}

void dump(const char *filename)
{
	int fd;
	Elf64_Ehdr *elf_header;
	Elf64_Phdr *program_headers;
	int essential_flags = O_WRONLY | O_CREAT;
	int harmless_paranoid_flags = O_EXCL | O_CLOEXEC;
	int open_flags = essential_flags | harmless_paranoid_flags;
	fd = open(filename, open_flags, 0777);
	if (fd < 0)
		fatal("%s: open: %s\n", filename, strerror(errno));

	elf_header = calloc(1, sizeof(*elf_header));
	if (elf_header == NULL)
		fatal("elf_header: out of memory\n");

	/* Layout of core file:
	 *
	 *    ==========
	 *    ELF header
	 *    ==========
	 *    Program Header 0 (PT_NOTE, for registers etc., see below)
	 *    Program Header 1 (PT_LOAD, for stack)
	 *    =========
	 *    Data for Program Header 0:
	 *      ----
	 *      Note Header for PRPSINFO (process info)
	 *      "CORE\0\0\0\0"
	 *      prpsinfo structure
	 *      ----
	 *      Note Header for PRSTATUS (includes GP registers)
	 *      "CORE\0\0\0\0"
	 *      prstatus structure
	 *      ----
	 *      Note Header for FPREGSET (FP registers)
	 *      "CORE\0\0\0\0"
	 *      fpregs structure
	 *    ======
	 *    Data for Program Header 1:
	 *      raw stack content
	 */

	/* Allocate one PT_NOTE and one PT_LOAD header. */
	const int program_headers_count = 2;
	program_headers = calloc(program_headers_count,
				 sizeof(*program_headers));
	if (program_headers == NULL)
		fatal("program_headers: out of memory\n");

	/* The PT_NOTE section contains registers and other info and is made of
	 * several subsections, each prefixed by a note header.  We put these
	 * subsections in the notes_data array.
	 */
	program_headers[0].p_type = PT_NOTE;

	char notes_data[4096];
	void *data = notes_data;

	struct elf_prpsinfo prps;
	memset(&prps, 0, sizeof(prps));
	data = add_note(data, NT_PRPSINFO, &prps, sizeof(prps));

	struct elf_prstatus prstatus;
	memset(&prstatus, 0, sizeof(prstatus));
	int signal = 0;
	prstatus.pr_info.si_signo = signal;
	prstatus.pr_cursig = signal;
	prstatus.pr_pid = 0;
	((struct user_regs_struct *) &prstatus.pr_reg)->rip = rip;
	((struct user_regs_struct *) &prstatus.pr_reg)->rsp = rsp;
	((struct user_regs_struct *) &prstatus.pr_reg)->rbp = rbp;
	data = add_note(data, NT_PRSTATUS, &prstatus, sizeof(prstatus));

	struct user_fpregs_struct fpregs;
	memset(&fpregs, 0, sizeof(fpregs));
	/* TODO(semenzato): fill the FP registers. */
	data = add_note(data, NT_FPREGSET, &fpregs, sizeof(fpregs));

	program_headers[0].p_filesz = ((char *) data) - notes_data;

	/* Stack */
	program_headers[1].p_type = PT_LOAD;
	program_headers[1].p_align = 4096;
	program_headers[1].p_vaddr = rsp;
	program_headers[1].p_filesz = stack_index * sizeof(uint64_t);
	program_headers[1].p_memsz = stack_index * sizeof(uint64_t);

	elf_output(fd, elf_header,
		   program_headers, program_headers_count,
		   notes_data, stack);
}

int main(int argc, char **argv)
{
	program_name = argv[0];
	int opt_index = 0;
	int opt_error = false;

	for (;;) {
		static struct option options[] = {
			{"fixaddresses", no_argument, NULL, 'f'},
			{NULL, no_argument, NULL, 0}
		};
		int opt;
		opt = getopt_long(argc, argv, "f", options, &opt_index);
		if (opt == -1)
			break;
		switch (opt) {
		case 'f':
			kaslr_fix_addresses_flag = true;
			break;
		default:
			opt_error = true;
			break;
		}
	}
	if (optind < argc || opt_error)
		fatal("usage: %s [<options>] < <kernel-log-with-panic>\n"
		      "Options:\n"
		      "-f (--fixaddresses)"
		      "    attempt to fix code addresses when kaslr is used\n"
		      "Output goes to \"kcrash.core\"\n", program_name);
	stack_index = 0;
	yylex();
	if (stack_index == 0)
		fatal("input does not contain a crash dump (no stack)\n");
	if (kaslr_fix_addresses_flag)
		kaslr_fix_addresses();
	dump("kcrash.core");
	return 0;
}

void remove_unused_warnings(void)
{
	yyunput(0, NULL);
	input();
}
