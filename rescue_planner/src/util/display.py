#!/usr/bin/env python3
import sys
import matplotlib.pyplot as plt

def main():
	data = sys.stdin.read().strip().split()
	if not data:
		return
	it = iter(data)
	try:
		L = int(next(it))
		P = int(next(it))
	except StopIteration:
		return

	lines = []
	for _ in range(L):
		try:
			x1 = float(next(it)); y1 = float(next(it)); x2 = float(next(it)); y2 = float(next(it))
		except StopIteration:
			break
		lines.append(((x1, y1), (x2, y2)))

	points = []
	for _ in range(P):
		try:
			x = float(next(it)); y = float(next(it))
		except StopIteration:
			break
		points.append((x, y))

	fig, ax = plt.subplots()

	# draw lines
	for a, b in lines:
		ax.plot([a[0], b[0]], [a[1], b[1]], color='gray', linewidth=1)

	# draw points with different colors
	if points:
		xs = [p[0] for p in points]
		ys = [p[1] for p in points]
		colors = list(range(len(points)))
		cmap = plt.get_cmap('tab10')
		ax.scatter(xs, ys, c=colors, cmap=cmap, s=50, edgecolors='black')

	ax.set_aspect('equal', 'box')
	plt.show()

if __name__ == '__main__':
	main()

