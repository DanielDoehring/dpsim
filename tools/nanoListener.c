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



int listen(const char *url) {

    int sock;

    if ((sock = nn_socket(AF_SP, NN_SUB)) < 0) {
        fatal("nn_socket");
        // fprintf(stderr, "test");
    }

    // NN_SUB = socket has protocol level "publish and subscribe"
    // NN_SUB_SUBSCRIBE = connect as subscriber, i.e. receiver
    // "" = subscribe to all topics of socket
    if (nn_setsockopt(sock, NN_SUB, NN_SUB_SUBSCRIBE, "", 0) < 0) {
        fatal("nn_setsockopt");
        // fprintf(stderr, "test");
    }
    
    // connect socket to remote url
    if (nn_connect(sock, url) < 0) {
        fatal("nn_connect");
        // fprintf(stderr, "test");
    }

    // Continously listen to socket and print all incoming messages
    fprintf(stderr, "Listener up and running on URL = %s\n", url);

    for(;;) {
        void *buf = NULL;
    
        int bytes = nn_recv(sock, &buf, NN_MSG, 0);
        if (bytes < 0) {
            fatal("nn_recv");
            // fprintf(stderr, "test");
        }

        fprintf(stderr, "RECEIVED %s\n", buf);
        nn_freemsg(buf);
    }
}

int main(const int argc, const char **argv) {
    
    if (argc <= 1) {
        fprintf(stderr, "Abort: No URL given!...\n Usage: nanoListener <URL> \n");
        return 1;
    } 
    else {
        return(listen(argv[1]));
    }
}
