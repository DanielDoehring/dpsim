#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include <nanomsg/nn.h>
#include <nanomsg/pubsub.h>


// Error print function
void fatal (const char *func) {
    fprintf(stderr, "%s: %s\n", func, nn_strerror(nn_errno()));
}

int main(const int argc, const char **argv) {
    
    if (argc <= 2) {
        fprintf(stderr, "Abort: Not enough arguments...\n Usage: nanoListener <URL_IN> <URL_OUT> \n");
        return -1;
    } 
    else {        
        int sock_in, sock_out;
        char *buf = NULL;

        // IN socket
        if ((sock_in = nn_socket(AF_SP, NN_SUB)) < 0) {
            fatal("nn_socket");
        }

        if (nn_setsockopt(sock_in, NN_SUB, NN_SUB_SUBSCRIBE, "", 0) < 0) {
            fatal("nn_setsockopt");
        }

        if (nn_connect(sock_in, argv[1]) < 0) {
            fatal("nn_connect");
        }

        fprintf(stderr, "Receiving on %s\n", argv[1]);
        // OUT socket
        if ((sock_out = nn_socket(AF_SP, NN_PUB)) < 0) {
                fatal("nn_socket");
        }
        if (nn_bind(sock_out, argv[2]) < 0) {
                fatal("nn_bind");
        }
        fprintf(stderr, "Sending on %s\n", argv[2]);


        ////////////////////////////

        for(;;) {
            int bytes = nn_recv(sock_in, &buf, NN_MSG, 0);
            if (bytes < 0) {
                fatal("nn_recv");
            }
            fprintf(stderr, "RECEIVED %s (%i bytes)\n", buf, sizeof(buf));
            fprintf(stderr, "|||||||||||||||\n");

            printf("SENDING: %s\n", buf);
            bytes = nn_send(sock_out, buf, bytes, 0);
            
            if (bytes < 0) {
                    fatal("nn_send");
            }
            else {
                printf("%i bytes sent\n", bytes);
            }

            nn_freemsg(buf);

        }
        return 1;
    }
}
