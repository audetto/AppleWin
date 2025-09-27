#include "getopt.h"
#include <string.h>
#include <stdio.h>

// For Windows MSVC only

char *optarg = NULL;
int optind = 1, opterr = 1, optopt = '?';

static int optpos = 1;

static int
_parse_short(int argc, char * const argv[], const char *optstring)
{
    char c = argv[optind][optpos];
    const char *cp = strchr(optstring, c);

    if (!cp) {
        if (opterr)
            fprintf(stderr, "unknown option -- %c\n", c);
        if (argv[optind][++optpos] == '\0') {
            optind++;
            optpos = 1;
        }
        optopt = c;
        return '?';
    }
    if (cp[1] == ':') {
        if (argv[optind][optpos+1] != '\0') {
            optarg = &argv[optind][optpos+1];
            optind++;
        } else if (++optind < argc) {
            optarg = argv[optind++];
        } else {
            optopt = c;
            return (optstring[0] == ':') ? ':' : '?';
        }
        optpos = 1;
    } else {
        if (argv[optind][++optpos] == '\0') {
            optpos = 1;
            optind++;
        }
        optarg = NULL;
    }
    return c;
}

int getopt(int argc, char * const argv[], const char *optstring)
{
    if (optind >= argc || argv[optind][0] != '-' || argv[optind][1] == '\0')
        return -1;
    if (strcmp(argv[optind], "--") == 0) {
        optind++;
        return -1;
    }
    return _parse_short(argc, argv, optstring);
}

int getopt_long(int argc, char * const argv[], const char *optstring,
                const struct option *longopts, int *longindex)
{
    if (optind >= argc) return -1;

    if (argv[optind][0] == '-' && argv[optind][1] == '-') {
        const char *name = argv[optind] + 2;
        const char *eq = strchr(name, '=');
        size_t len = eq ? (size_t)(eq - name) : strlen(name);

        for (int i = 0; longopts[i].name; i++) {
            if (strncmp(name, longopts[i].name, len) == 0
                && len == strlen(longopts[i].name)) {
                if (longindex) *longindex = i;
                if (longopts[i].has_arg == no_argument) {
                    optarg = NULL;
                } else if (longopts[i].has_arg == required_argument) {
                    if (eq)
                        optarg = (char*)eq+1;
                    else if (optind+1 < argc)
                        optarg = argv[++optind];
                    else
                        return optopt = longopts[i].val, '?';
                } else if (longopts[i].has_arg == optional_argument) {
                    optarg = eq ? (char*)eq+1 : NULL;
                }
                optind++;
                if (longopts[i].flag) {
                    *(longopts[i].flag) = longopts[i].val;
                    return 0;
                } else {
                    return longopts[i].val;
                }
            }
        }
        /* no match */
        if (opterr)
            fprintf(stderr, "unrecognized option '--%s'\n", name);
        optind++;
        return '?';
    }
    /* fallback to short option parsing */
    return getopt(argc, argv, optstring);
}

int getopt_long_only(int argc, char * const argv[], const char *optstring,
                     const struct option *longopts, int *longindex)
{
    return getopt_long(argc, argv, optstring, longopts, longindex);
}
