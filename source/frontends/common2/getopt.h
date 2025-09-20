#ifndef GETOPT_H
#define GETOPT_H

// For Windows MSVC only

#ifdef __cplusplus
extern "C" {
#endif

/* argument flags */
#define no_argument        0
#define required_argument  1
#define optional_argument  2

struct option {
    const char *name;
    int  has_arg;
    int *flag;
    int  val;
};

/* globals */
extern char *optarg;
extern int optind, opterr, optopt;

/* functions */
int getopt(int argc, char * const argv[], const char *optstring);

int getopt_long(int argc, char * const argv[], const char *optstring,
                const struct option *longopts, int *longindex);

int getopt_long_only(int argc, char * const argv[], const char *optstring,
                     const struct option *longopts, int *longindex);

#ifdef __cplusplus
}
#endif

#endif /* GETOPT_H */
