#ifndef PE_H
#define PE_H

#include <iomanip>
#include <cmath>
#include "angle_table.h"

#include <systemc>
using namespace sc_core;

#include <tlm>
#include <tlm_utils/simple_target_socket.h>

#include "defines.h"

class PE : public sc_module {
    public:
        tlm_utils::simple_target_socket<PE> tsock;

        sc_fifo<data_t> i_a;
        sc_fifo<data_t> i_b;
        sc_fifo<angle_t> i_z;
        sc_fifo<data_t> o_a;
        sc_fifo<data_t> o_b;
        sc_fifo<angle_t> o_z;

        SC_HAS_PROCESS( PE );

        PE(sc_module_name n) : sc_module(n), tsock("t_skt"), base_offset(0) {
            tsock.register_b_transport(this, &PE::blocking_transport);
            SC_THREAD(comp);
        }

        ~PE() = default;

    private:
        unsigned int base_offset;

        CORDIC_output_t CORDIC_one(data_t x_0, data_t y_0, angle_t theta_0, bool mode, unsigned int SFL, angle_t theta_const) {
            data_t tmp_x;
            data_t tmp_y;
            angle_t tmp_theta;
            data_t x;
            data_t y;
            angle_t theta;
            CORDIC_output_t o_data;

            tmp_x = x_0 >> SFL;
            tmp_y = y_0 >> SFL;
            if (mode) { // vectoring mode
                if (y > 0) { // rotate negative degree
                    tmp_x = -tmp_x;
                    tmp_theta = theta_const;
                } else {     // rotate positive degree
                    tmp_y = -tmp_y;
                    tmp_theta = -theta_const;
                }
            } else { // rotation mode
                if (theta_0 > 0) { // rotate positive degree
                    tmp_y = -tmp_y;
                    tmp_theta = -theta_const;
                } else {           // rotate negative degree
                    tmp_x = -tmp_x;
                    tmp_theta = theta_const;
                }
            }
            x = x_0 + tmp_y;
            y = tmp_x + y_0; 
            theta = theta_0 + tmp_theta;
            o_data.x = x;
            o_data.y = y;
            o_data.theta = theta;
            return o_data;
        }

        CORDIC_output_t CORDIC(data_t x_0, data_t y_0, angle_t theta_0, bool mode) {
            data_t cor_x;
            data_t cor_y;
            angle_t cor_theta;
            CORDIC_output_t CORDIC_out;
            data_t factor = 0.6072776441;    
        
            // input correction
            if (mode) {   // vectoring mode
                if (y_0 > 0) {
                    cor_x = y_0; // rotate -90 degree
                    cor_y = -x_0;
                    cor_theta = theta_0 + 90.0;
                } else {
                    cor_x = -y_0; // rotate 90 degree
                    cor_y = x_0;  
                    cor_theta = theta_0 - 90.0;
                }
            } else {     // rotation mode
                if(theta_0 > 0) {
                    cor_x = -y_0; // rotate 90 degree
                    cor_y = x_0;  
                    cor_theta = theta_0 - 90.0;
                } else {
                    cor_x = y_0;  // rotate -90 degree
                    cor_y = -x_0;
                    cor_theta = theta_0 + 90.0;
                }
            }
            // computation
            CORDIC_out = CORDIC_one(cor_x, cor_y, cor_theta, mode, 0, ANGLE0);
            CORDIC_out = CORDIC_one(CORDIC_out.x, CORDIC_out.y, CORDIC_out.theta, mode, 1, ANGLE1);
            CORDIC_out = CORDIC_one(CORDIC_out.x, CORDIC_out.y, CORDIC_out.theta, mode, 2, ANGLE2);
            CORDIC_out = CORDIC_one(CORDIC_out.x, CORDIC_out.y, CORDIC_out.theta, mode, 3, ANGLE3);
            CORDIC_out = CORDIC_one(CORDIC_out.x, CORDIC_out.y, CORDIC_out.theta, mode, 4, ANGLE4);
            CORDIC_out = CORDIC_one(CORDIC_out.x, CORDIC_out.y, CORDIC_out.theta, mode, 5, ANGLE5);
            CORDIC_out = CORDIC_one(CORDIC_out.x, CORDIC_out.y, CORDIC_out.theta, mode, 6, ANGLE6);
            CORDIC_out.x = CORDIC_out.x * factor;
            CORDIC_out.y = CORDIC_out.y * factor;
            return CORDIC_out; 
        }

        void comp() {
            while (true) {
                data_t input_a;
                data_t input_b;
                angle_t input_z;
                data_t output_a;
                data_t output_b;
                angle_t output_z;
                CORDIC_output_t CORDIC_out;
                {
                    // input
                    input_a = i_a.read();
                    input_b = i_b.read();
                    input_z = i_z.read();
                }
                // computation
                CORDIC_out = CORDIC(input_a, input_b, input_z, false);
                output_a = CORDIC_out.x;
                output_b = CORDIC_out.y;
                output_z = CORDIC_out.theta;
                {
                    // output
                    o_a.write(output_a);
                    o_b.write(output_b);
                    o_z.write(output_z);
                }
            }
        }

    void blocking_transport(tlm::tlm_generic_payload &payload, sc_core::sc_time &delay){
        wait(delay);
        // unsigned char *mask_ptr = payload.get_byte_enable_ptr();
        // auto len = payload.get_data_length();
        tlm::tlm_command cmd = payload.get_command();
        sc_dt::uint64 addr = payload.get_address();
        unsigned char *data_ptr = payload.get_data_ptr();

        addr -= base_offset;

        word data;

        switch (cmd) {
        case tlm::TLM_READ_COMMAND:
            // cout << "READ" << endl;
            switch (addr) {
            case PE_OUTPUT_A_ADDR:
                data.f = (float) o_a.read();
                break;
            case PE_OUTPUT_B_ADDR:
                data.f = (float) o_b.read();
                break;
            case PE_OUTPUT_Z_ADDR:
                data.f = (float) o_z.read();
                break;
            default:
                std::cerr << "READ Error! PE::blocking_transport: address 0x"
                        << std::setfill('0') << std::setw(8) << std::hex << addr
                        << std::dec << " is not valid" << std::endl;
            }
            for (int i = 0; i < 4; ++i) {
                data_ptr[i] = data.uc[i];
            }
            break;
        case tlm::TLM_WRITE_COMMAND:
            // cout << "WRITE" << endl;
            for (int i = 0; i < 4; ++i) {
                data.uc[i] = data_ptr[i];
            }
            switch (addr) {
            case PE_INPUT_A_ADDR:
                i_a.write((data_t) data.f);
                break;
            case PE_INPUT_B_ADDR:
                i_b.write((data_t) data.f);
                break;
            case PE_INPUT_Z_ADDR:
                i_z.write((angle_t) data.f);
                break;
            default:
                std::cerr << "WRITE Error! SobelFilter::blocking_transport: address 0x"
                        << std::setfill('0') << std::setw(8) << std::hex << addr
                        << std::dec << " is not valid" << std::endl;
            }
            break;
        case tlm::TLM_IGNORE_COMMAND:
            payload.set_response_status(tlm::TLM_GENERIC_ERROR_RESPONSE);
            return;
        default:
            payload.set_response_status(tlm::TLM_GENERIC_ERROR_RESPONSE);
            return;
        }
        payload.set_response_status(tlm::TLM_OK_RESPONSE); // Always OK
    }
};

#endif
