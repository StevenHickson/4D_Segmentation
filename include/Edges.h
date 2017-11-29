/*
Copyright (C) 2012 Steven Hickson

MIT License
*/

#ifndef EDGES_H
#define EDGES_H

class Edge {
public:
	float w;
	int a, b;
	bool valid;
	Edge() : w(0), a(0), b(0), valid(false) { };

	bool operator< (const Edge &other) const {
		return (w < other.w);
	}
};

class Edge3D : public Edge {
public:
	float w2;

	Edge3D() :  Edge(), w2(0) { };
};


#endif
