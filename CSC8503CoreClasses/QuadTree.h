﻿#pragma once
#include "CollisionDetection.h"
#include "Debug.h"

namespace NCL {
	using namespace NCL::Maths;
	namespace CSC8503 {
		template<class T>
		class QuadTree;

		template<class T>
		struct QuadTreeEntry {
			Vector3 pos;
			Vector3 size;
			T object;

			QuadTreeEntry(T obj, Vector3 pos, Vector3 size) {
				object		= obj;
				this->pos	= pos;
				this->size	= size;
			}
		};

		template<class T>
		class QuadTreeNode	{
		public:
			typedef std::function<void(std::list<QuadTreeEntry<T>>&)> QuadTreeFunc;

		protected:
			friend class QuadTree<T>;

			QuadTreeNode() {}

			QuadTreeNode(Vector2 pos, Vector2 size) {
				children		= nullptr;
				this->position	= pos;
				this->size		= size;
			}

			~QuadTreeNode() {
				delete[] children;
			}

			void Insert(T& object, const Vector3& objectPos, const Vector3& objectSize, int depthLeft, int maxSize) {
				if (!CollisionDetection::AABBTest(objectPos, Vector3(position.x, 0, position.y), objectSize, Vector3(size.x, 1000.0f, size.y))) {
					return;
				}
				if (children) {
					for (int i = 0; i < 4; ++i) {
						children[i].Insert(object, objectPos, objectSize, depthLeft - 1, maxSize);
					}
				}
				else {
					contents.push_back(QuadTreeEntry<T>(object, objectPos, objectSize));
					//如果当前节点存储的内容超过最大值，且深度大于0，则进行分裂
					if ((int)contents.size() > maxSize && depthLeft > 0) {
						if (!children) {
							Split();
							for (const auto& i : contents) {
								for (int j = 0; j < 4; ++j) {
									auto entry = i;
									children[j].Insert(entry.object, entry.pos, entry.size, depthLeft - 1, maxSize);
								}
							}
							contents.clear();
						}
					}
				}
			}

			void Split() {
				Vector2 halfSize = size / 2.0f;
				children = new QuadTreeNode<T>[4];
				//左上，右上，左下，右下
				children[0] = QuadTreeNode<T>(position + Vector2(-halfSize.x, halfSize.y), halfSize);
				children[1] = QuadTreeNode<T>(position + Vector2(halfSize.x, halfSize.y), halfSize);
				children[2] = QuadTreeNode<T>(position + Vector2(-halfSize.x, -halfSize.y), halfSize);
				children[3] = QuadTreeNode<T>(position + Vector2(halfSize.x, -halfSize.y), halfSize);
			}

			void DebugDraw() {
			}

			void OperateOnContents(QuadTreeFunc& func) {
				if (children) {
					//四叉树，因此有四个子节点
					for (int i = 0; i < 4; ++i) {
						//如果存在子节点，说明对象一定在子节点中，为子节点执行操作
						children[i].OperateOnContents(func);
					}
				}
				else {
					//不存在子节点且存储内容不为空时，为当前节点执行操作
					if (!contents.empty())
						func(contents);
				}
			}

		protected:
			std::list< QuadTreeEntry<T> >	contents;

			Vector2 position;
			Vector2 size;

			QuadTreeNode<T>* children;
		};
	}
}


namespace NCL {
	using namespace NCL::Maths;
	namespace CSC8503 {
		template<class T>
		class QuadTree
		{
		public:
			QuadTree(Vector2 size, int maxDepth = 6, int maxSize = 5){
				root = QuadTreeNode<T>(Vector2(), size);
				this->maxDepth	= maxDepth; //允许分裂的最大深度
				this->maxSize	= maxSize; //每个节点允许存储的最大对象数量
			}
			~QuadTree() {
			}

			void Insert(T object, const Vector3& pos, const Vector3& size) {
				root.Insert(object, pos, size, maxDepth, maxSize);
			}

			void DebugDraw() {
				root.DebugDraw();
			}

			void OperateOnContents(typename QuadTreeNode<T>::QuadTreeFunc  func) {
				root.OperateOnContents(func);
			}

		protected:
			QuadTreeNode<T> root;
			int maxDepth;
			int maxSize;
		};
	}
}