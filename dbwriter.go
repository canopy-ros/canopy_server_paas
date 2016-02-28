package main

import (
    "github.com/garyburd/redigo/redis"
)

type command struct {
    comm string
    args []interface{}
    reply chan interface{}
    err chan error
}

type dbwriter struct {
    redisconn *redis.Conn
    commChannel chan command
}

func (d *dbwriter) write(comm string, args ...interface{}) (interface{}, error) {
    c := command{comm: comm, args: args, reply: make(chan interface{}), err: make(chan error)}
    d.commChannel <- c
    res := <- c.reply
    err := <- c.err
    return res, err
}

func (d *dbwriter) writer() {
    for c := range d.commChannel {
        reply, err := (*d.redisconn).Do(c.comm, c.args...)
        c.reply <- reply
        c.err <- err
    }
}
