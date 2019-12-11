/* Helper program that simulates a DistAIX simulation of a superior grid for testing Dist2Dist communication.
 *
 * ./helper <url_of_in_endpoint> <url_of_out_endpoint>
 * 
 * <url_of_in_endpoint> = url of distaix simulation host
 * <url_of_out_endpoint> = url of helper tool host
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <stdbool.h>

#include <nanomsg/nn.h>
#include <nanomsg/pubsub.h>

int in_socket;
int out_socket;
union Data {
	int i;
	char* ch;
	float f;
	bool b;
};

union Data a_test_message[8];

char* test_message[] = {"1"};
void fatal(const char *func) {
        fprintf(stderr, "%s: %s\n", func, nn_strerror(nn_errno()));
}

char * date(void) {
        time_t now = time(&now);
        struct tm *info =localtime(&now);
        char *text = asctime(info);
        text[strlen(text)-1] = '\0'; // remove '\n'
        return (text);
}
void create_message(){
	union Data d1,d2,d3,d4,d5,d6,d7,d8;	

	d1.i = 47;
	d2.i = 1;
	d3.f = 2.22;
	d4.f = 3.33;
	d5.f = 4.44;
	d6.f = 5.55;
	d7.i = 1;	
	d8.i = 4;
	
	a_test_message[0] = d1;
	a_test_message[1] = d2;
	a_test_message[2] = d3;
	a_test_message[3] = d4;
	a_test_message[4] = d5;
	a_test_message[5] = d6;
	a_test_message[6] = d7;
	a_test_message[7] = d8;
}

/* 
        Function that sends a message onto out_socket
*/
int send(char* buf, int length) {
        int ret = 1;

        //char *message = date();
        //int length_message = strlen(test_message) + 1;
        //char *buf = &a_test_message;
	printf("SENDING: %s\n", buf);
        int bytes = nn_send(out_socket, buf, length, 0);
        
        if (bytes < 0) {
                fatal("nn_send");
                ret = 0;
        }
	else {
		printf("%i bytes sent\n", bytes);
	}

        return ret;
}

/*
        Function that receives a message from in_socket
*/
char* receive(int *bytes) {
        int ret = 1;
        
        //int bytes;
	
        char *buf = NULL;

        printf("Waiting for message...\n");
        // nn_recv() uses busy waiting by default
        if ((*bytes = nn_recv(in_socket, &buf, NN_MSG, 0)) < 0) {
                fatal("nn_recv");
                ret = 0;
	}
        printf("RECEIVED: %s (%i bytes)\n", buf,*bytes);
        //nn_freemsg(buf);

        return buf;
}

/*
        Function that realizes setup of communication

        url_in = adress of endpoint used for receiving
        url_out = adress of endpoint used for sending
*/
int setup(const char *url_in, const char *url_out) {
        int ret = 1;

        ///////////////////////////////////////////////
        // Configure and bind out_socket
        ///////////////////////////////////////////////
        
        if ((out_socket = nn_socket(AF_SP, NN_PUB)) < 0) {
                fatal("nn_socket");
                ret = 0;
        }

        if (nn_bind(out_socket, url_out) < 0) {
                fatal("nn_bind");
                ret = 0;
        }

        ///////////////////////////////////////////////
        // Configure and connect in_socket
        ///////////////////////////////////////////////
        
        if ((in_socket = nn_socket(AF_SP, NN_SUB)) < 0) {
                fatal("nn_socket");
                ret = 0;
        }
        
        // "" --> subscribe to ALL topics
        if (nn_setsockopt(in_socket, NN_SUB, NN_SUB_SUBSCRIBE, "", 0) < 0) {
                fatal("nn_setsockopt");
                ret = 0;
        }
        
        if (nn_connect(in_socket, url_in) < 0) {
                fatal("nn_connect");
                ret = 0;
        }

        return ret;

}

/*      Main-routine:
                1) Setup the communication sockets
                2) Wait (busy) until message received
                3) Delay
                4) Send message over out_socket
                5) Goto 2)
*/
int main(const int argc, const char **argv) {
	if ((argc >= 2)) {
                if (setup(argv[1], argv[2]) != 1) {
                        printf("Error: Setup failed!\n");
                }
        }
        else {
                printf("Error: Not enough parameters!\n");
        }
	// create_message();	
        // char* buffer = NULL;
	// int *bytes;
	// *bytes = 0;

	// for(;;) {
        //         buffer = receive(bytes);
	// 	//sleep(5);
        //         send(buffer, *bytes);
        // }
	// nn_freemsg(buffer);
        // return 1;
        
        char message[] = "superMessage69";

        for(;;) {
                send(message, sizeof(a_test_message));
                sleep(5);
        }
}
