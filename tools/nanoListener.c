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



int listen(const char *url, const char *filepath) {

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
 
        if (filepath != ""){
            FILE* pFile = fopen(filepath, "wb");
            if(pFile){
                fwrite(buf, bytes, 1, pFile);
                fprintf(stderr, "Wrote buffer to file: %s\n", filepath);
            }
            else {
                fprintf(stderr, "Could not open file %s\n", filepath);
            }
            fclose(pFile);
        }

        nn_freemsg(buf);
    }
}

int main(const int argc, const char **argv) {
    
    // Option to set a filepath. If path is set != "", the buffer will be written to the corresponding file.
    //const char* filepath="tools/complexValue.buf";
    const char* filepath = "";
    
    if (argc <= 1) {
        fprintf(stderr, "Abort: No URL given!...\n Usage: nanoListener <URL> \n");
        return 1;
    } 
    else {
        return(listen(argv[1], filepath));
    }
}
