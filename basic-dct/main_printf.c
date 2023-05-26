#include "stdio.h"
#include "stdlib.h"
#include "stdbool.h"
#include "string.h"
#include "math.h"
#include "stdint.h"
// #include "irq.h"

#define PI 3.1415926535097

// Sobel Filter ACC
static char* const PE1_START_ADDR   = (char* const)0x03000000;
static char* const PE1_OUTPUT_ADDR  = (char* const)0x0300000c;
static char* const PE2_START_ADDR   = (char* const)0x03100000;
static char* const PE2_OUTPUT_ADDR  = (char* const)0x0310000c;

// DMA 
static volatile uint32_t * const DMA_SRC_ADDR  = (uint32_t * const)0x70000000;
static volatile uint32_t * const DMA_DST_ADDR  = (uint32_t * const)0x70000004;
static volatile uint32_t * const DMA_LEN_ADDR  = (uint32_t * const)0x70000008;
static volatile uint32_t * const DMA_OP_ADDR   = (uint32_t * const)0x7000000C;
static volatile uint32_t * const DMA_STAT_ADDR = (uint32_t * const)0x70000010;
static const uint32_t DMA_OP_NOP = 0;
static const uint32_t DMA_OP_MEMCPY = 1;

int _is_using_dma = 1;
// volatile int dma_completed = 0;

union pack {
  float f[3];
  unsigned char uc[12];
};

// void dma_irq_handler() {
// 	dma_completed = 1;
// }

// void init() {
// 	register_interrupt_handler(4, dma_irq_handler);
// }


int sem_init (uint32_t *__sem, uint32_t count) __THROW
{
  *__sem=count;
  return 0;
}

int sem_wait (uint32_t *__sem) __THROW
{
  uint32_t value, success; //RV32A
  __asm__ __volatile__("\
L%=:\n\t\
     lr.w %[value],(%[__sem])            # load reserved\n\t\
     beqz %[value],L%=                   # if zero, try again\n\t\
     addi %[value],%[value],-1           # value --\n\t\
     sc.w %[success],%[value],(%[__sem]) # store conditionally\n\t\
     bnez %[success], L%=                # if the store failed, try again\n\t\
"
    : [value] "=r"(value), [success]"=r"(success)
    : [__sem] "r"(__sem)
    : "memory");
  return 0;
}

int sem_post (uint32_t *__sem) __THROW
{
  uint32_t value, success; //RV32A
  __asm__ __volatile__("\
L%=:\n\t\
     lr.w %[value],(%[__sem])            # load reserved\n\t\
     addi %[value],%[value], 1           # value ++\n\t\
     sc.w %[success],%[value],(%[__sem]) # store conditionally\n\t\
     bnez %[success], L%=                # if the store failed, try again\n\t\
"
    : [value] "=r"(value), [success]"=r"(success)
    : [__sem] "r"(__sem)
    : "memory");
  return 0;
}

int barrier(uint32_t *__sem, uint32_t *__lock, uint32_t *counter, uint32_t thread_count) {
	sem_wait(__lock);
	if (*counter == thread_count - 1) { //all finished
		*counter = 0;
		sem_post(__lock);
		for (int j = 0; j < thread_count - 1; ++j) sem_post(__sem);
	} else {
		(*counter)++;
		sem_post(__lock);
		sem_wait(__sem);
	}
	return 0;
}

void write_data_to_ACC(char* ADDR, unsigned char* buffer, int len){
  if(_is_using_dma){  
    // Using DMA 
	// dma_completed = 0;
    *DMA_SRC_ADDR = (uint32_t)(buffer);
    *DMA_DST_ADDR = (uint32_t)(ADDR);
    *DMA_LEN_ADDR = len;
    *DMA_OP_ADDR  = DMA_OP_MEMCPY;
	// while (!dma_completed) {
	// 	asm volatile ("wfi");
	// }
  }else{
    // Directly Send
    memcpy(ADDR, buffer, sizeof(unsigned char)*len);
  }
}
void read_data_from_ACC(char* ADDR, unsigned char* buffer, int len){
  if(_is_using_dma){
    // Using DMA 
	// dma_completed = 0;
    *DMA_SRC_ADDR = (uint32_t)(ADDR);
    *DMA_DST_ADDR = (uint32_t)(buffer);
    *DMA_LEN_ADDR = len;
    *DMA_OP_ADDR  = DMA_OP_MEMCPY;
	// while (!dma_completed) {
	// 	asm volatile ("wfi");
	// }
  }else{
    // Directly Read
    memcpy(buffer, ADDR, sizeof(unsigned char)*len);
  }
}

float phase_correction(float phase) {
	float new_phase = phase;
	while (new_phase > 180.0) new_phase = new_phase - 360.0;
	while (new_phase < -180.0) new_phase = new_phase + 360.0;
	return new_phase;
}

// Total number of cores
// static const int PROCESSORS = 2;
#define PROCESSORS 2
// the barrier synchronization objects
uint32_t barrier_counter=0; 
uint32_t barrier_lock; 
uint32_t barrier_sem; 
// the mutex object to control global summation
uint32_t lock;  
// print synchronication semaphore (print in core order)
uint32_t print_sem[PROCESSORS]; 
// global memory
float **input_memory;
float **output_memory;
int n, m;
// file pointer
FILE *input_fptr;
FILE *output_fptr;

int main(unsigned hart_id) {
	/////////////////////////////
	// thread and barrier init //
	/////////////////////////////
	if (hart_id == 0) {
		// create a barrier object with a count of PROCESSORS
		sem_init(&barrier_lock, 1);
		sem_init(&barrier_sem, 0); //lock all cores initially
		for(int i=0; i< PROCESSORS; ++i){
			sem_init(&print_sem[i], 0); //lock printing initially
		}
		// Create mutex lock
		sem_init(&lock, 1);
	}

	/////////////////////////////////
	// Open input and output file  //
	/////////////////////////////////
	if (hart_id == 0) {
		input_fptr = fopen("dct_testcase.txt", "r");
		output_fptr = fopen("dct_out.txt", "w");
	}

	/////////////////////////////////
	// 			Read file          //
	/////////////////////////////////
	if (hart_id == 0) {
		fscanf(input_fptr, "%d %d", &n, &m);
		input_memory = malloc(n * sizeof(float *));
		output_memory = malloc(n * sizeof(float *));
		for (int i = 0; i < n; ++i) {
			input_memory[i] = malloc(m * sizeof(float));
			output_memory[i] = malloc(m * sizeof(float));
			for (int j = 0; j < m; ++j) {
				fscanf(input_fptr, "%f", &input_memory[i][j]);
				output_memory[i][j] = 0;
			}
		}
		fclose(input_fptr);
	}
	/////////////////////////////////
	//  Read file Synchronization  //
	/////////////////////////////////
	if (hart_id == 0) sem_post(&print_sem[1]);
	else sem_wait(&print_sem[1]); 

	/////////////////////////////////
	//       Computation           //
	/////////////////////////////////
	unsigned char  buffer[12] = {0};
  	union pack data;
	float phase;
	for (int i = 0; i < n; ++i) {
		for (int j = 0; j < m; ++j) {
			float local_sum = 0;
			for (int k = hart_id; k < m; k = k + PROCESSORS) {	
				/////////////////////////
				// calculate local sum //
				/////////////////////////
				// preoare data for cosine evaluation
				data.f[0] = 1.0;
				data.f[1] = 0.0;
				phase = 180.0 * (k + 0.5) * j / m;
				data.f[2] = phase_correction(phase);
				for (int l = 0; l < 12; l++) buffer[l] = data.uc[l];
				// write data to PE
				sem_wait(&lock);
				if (hart_id == 0) write_data_to_ACC(PE1_START_ADDR, buffer, 12);
				else write_data_to_ACC(PE2_START_ADDR, buffer, 12);
				sem_post(&lock);
				sem_wait(&lock);
				if (hart_id == 0) read_data_from_ACC(PE1_OUTPUT_ADDR, buffer, 12);
				else read_data_from_ACC(PE2_OUTPUT_ADDR, buffer, 12);
				sem_post(&lock);
				// accumulate local sum
				for (int l = 0; l < 12; l++) data.uc[l] = buffer[l];
				local_sum = local_sum + input_memory[i][k] * data.f[0];
			}
			if (j == 0) local_sum = local_sum / sqrt(m);
			else local_sum = local_sum * sqrt(2.0 / m);
			/////////////////////////////////////////
			// accumulate local sum to shared data //
			/////////////////////////////////////////
			sem_wait(&lock);
			output_memory[i][j] = output_memory[i][j] + local_sum;
			sem_post(&lock);
		}
	}

	////////////////////////////
	// barrier to synchronize //
	////////////////////////////
	//Wait for all threads to finish
	barrier(&barrier_sem, &barrier_lock, &barrier_counter, PROCESSORS);

	if (hart_id == 0) {  // Core 0 print first and then others
		printf("core%d is finished\n", hart_id);
		sem_post(&print_sem[1]);  // Allow Core 1 to print
	} else {
			sem_wait(&print_sem[1]); 
			printf("core%d, finished\n", hart_id);
			for (int i = 0; i < n; ++i) {
				for (int j = 0; j < m; ++j) {
					fprintf(output_fptr, "%g ", output_memory[i][j]);
				}
				fprintf(output_fptr, "\n");
			}
			for (int i = 0; i < n; ++i) {
				free(input_memory[i]);
				free(output_memory[i]);
			}
			free(input_memory);
			free(output_memory);
			fclose(output_fptr);
	}

	return 0;
}
