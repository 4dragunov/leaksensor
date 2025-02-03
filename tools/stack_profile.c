static unsigned long long max_stack_usage = 0;

#define __GET_CURRENT_STACK_POINTER() __get_SP()

void __cyg_profile_func_enter(void * this, void * call) __attribute__((no_instrument_function)) {
      // get current stack usage using some hardware facility or intrisic function
      // like __get_SP() on ARM with CMSIS
      unsigned cur_stack_usage = __GET_CURRENT_STACK_POINTER() - __GET_BEGINNING_OF_STACK();
      // use debugger to output current stack pointer
      // for example semihosting on ARM
      __DEBUGGER_TRANSFER_STACK_POINTER(cur_stack_usage);
      // or you could store the max somewhere
      // then just run the program
      if (max_stack_usage < cur_stack_usage) {
            max_stack_usage = max_stack_usage;
      }
      // you could also manually inspect with debugger
      unsigned long long somelimit = 0x2000;
      if (cur_stack_usage > somelimit) {
           __BREAKPOINT();
      }
}
void __cyg_profile_func_exit(void * this, void * call) __attribute__((no_instrument_function)) {
      // well, nothing
}
