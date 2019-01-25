#pragma once

#include <vector>
#include <algorithm>

class UnionFind {
public:
	UnionFind(const size_t size) : data(size, -1) {}

	bool find_set(int x, int y) {
		return root(x) == root(y);
	}

	bool union_set(int x, int y) {
		x = root(x);
		y = root(y);
		if (x != y) {
			if (data[y] < data[x]) std::swap(x, y);
			data[x] += data[y];
			data[y] = x;
		}
		return x != y;
	}

	int root(int x) {
		return data[x] < 0 ? x : data[x] = root(data[x]);
	}

	int size(int x) {
		return -data[root(x)];
	}

	/**
	 * 一番データが多い結合のルート番号を返します.
	 */
	int maxsize_root() {
		int maxsize = 0;
		int maxindx = 0;
		for (int n = 0; n < (int) data.size(); n++) {
			if (data[n] < 0) {
				int size = -data[n];
				if (maxsize < size) {
					maxsize = size;
					maxindx = n;
				}
			}
		}
		return maxindx;
	}


public:
	std::vector<int> data;	// 負だったらroot.絶対値でそのグループに属する点の個数を表す. 正だったら親へのインデックス
};
