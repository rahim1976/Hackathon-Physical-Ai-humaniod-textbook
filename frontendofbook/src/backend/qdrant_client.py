"""
Qdrant client utilities for the RAG Chatbot
Provides enhanced functionality for vector storage operations.
"""

import logging
from typing import List, Dict, Any, Optional
from qdrant_client import QdrantClient
from qdrant_client.http import models
from qdrant_client.models import PointStruct, VectorParams, Distance

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class EnhancedQdrantClient:
    """
    Enhanced Qdrant client with additional functionality for the RAG system.
    """

    def __init__(self, url: str, api_key: str, prefer_grpc: bool = True):
        """
        Initialize the enhanced Qdrant client.

        Args:
            url: Qdrant server URL
            api_key: Qdrant API key
            prefer_grpc: Whether to prefer gRPC for communication
        """
        self.client = QdrantClient(
            url=url,
            api_key=api_key,
            prefer_grpc=prefer_grpc
        )

    def create_collection_with_config(self,
                                    collection_name: str,
                                    vector_size: int = 1536,
                                    distance: Distance = Distance.COSINE,
                                    hnsw_config: Optional[Dict[str, Any]] = None,
                                    optimizers_config: Optional[Dict[str, Any]] = None) -> bool:
        """
        Create a collection with specific configuration.

        Args:
            collection_name: Name of the collection to create
            vector_size: Size of the vectors
            distance: Distance metric to use
            hnsw_config: HNSW index configuration
            optimizers_config: Optimizers configuration

        Returns:
            True if collection was created successfully
        """
        try:
            # Set default configurations if not provided
            if hnsw_config is None:
                hnsw_config = {
                    "m": 16,
                    "ef_construct": 100,
                    "full_scan_threshold": 10000
                }

            if optimizers_config is None:
                optimizers_config = {
                    "deleted_threshold": 0.2,
                    "vacuum_min_vector_number": 1000
                }

            # Create collection
            self.client.recreate_collection(
                collection_name=collection_name,
                vectors_config=VectorParams(
                    size=vector_size,
                    distance=distance
                ),
                hnsw_config=hnsw_config,
                optimizers_config=optimizers_config
            )

            logger.info(f"Collection '{collection_name}' created successfully")
            return True

        except Exception as e:
            logger.error(f"Error creating collection '{collection_name}': {str(e)}")
            return False

    def batch_upsert_points(self,
                           collection_name: str,
                           points: List[PointStruct],
                           batch_size: int = 100) -> bool:
        """
        Upsert points in batches for better performance.

        Args:
            collection_name: Name of the collection
            points: List of PointStruct objects to upsert
            batch_size: Number of points to process in each batch

        Returns:
            True if all points were upserted successfully
        """
        try:
            total_points = len(points)
            logger.info(f"Upserting {total_points} points in batches of {batch_size}")

            for i in range(0, total_points, batch_size):
                batch = points[i:i + batch_size]
                self.client.upsert(
                    collection_name=collection_name,
                    points=batch
                )
                logger.info(f"Upserted batch {i//batch_size + 1}/{(total_points-1)//batch_size + 1}")

            logger.info(f"Successfully upserted {total_points} points")
            return True

        except Exception as e:
            logger.error(f"Error upserting points: {str(e)}")
            return False

    def search_with_filters(self,
                           collection_name: str,
                           query_vector: List[float],
                           query_filter: Optional[models.Filter] = None,
                           limit: int = 10,
                           offset: int = 0,
                           with_payload: bool = True,
                           with_vectors: bool = False,
                           score_threshold: Optional[float] = None) -> List[models.ScoredPoint]:
        """
        Search with additional filtering options.

        Args:
            collection_name: Name of the collection to search
            query_vector: Query vector for similarity search
            query_filter: Filter conditions for the search
            limit: Maximum number of results to return
            offset: Number of results to skip
            with_payload: Whether to return payload data
            with_vectors: Whether to return vector data
            score_threshold: Minimum similarity score threshold

        Returns:
            List of scored points matching the search criteria
        """
        try:
            results = self.client.search(
                collection_name=collection_name,
                query_vector=query_vector,
                query_filter=query_filter,
                limit=limit,
                offset=offset,
                with_payload=with_payload,
                with_vectors=with_vectors,
                score_threshold=score_threshold
            )

            logger.info(f"Search returned {len(results)} results")
            return results

        except Exception as e:
            logger.error(f"Error during search: {str(e)}")
            return []

    def get_collection_info(self, collection_name: str) -> Optional[Dict[str, Any]]:
        """
        Get detailed information about a collection.

        Args:
            collection_name: Name of the collection

        Returns:
            Dictionary with collection information or None if collection doesn't exist
        """
        try:
            info = self.client.get_collection(collection_name)
            return {
                "name": info.config.params.vectors.size,
                "vector_size": info.config.params.vectors.size,
                "distance": info.config.params.vectors.distance,
                "point_count": info.points_count,
                "indexed_vectors_count": info.indexed_vectors_count,
                "config": info.config.dict()
            }
        except Exception as e:
            logger.error(f"Error getting collection info: {str(e)}")
            return None

    def delete_points_by_payload_filter(self,
                                       collection_name: str,
                                       key: str,
                                       match_value: Any) -> bool:
        """
        Delete points that match a specific payload filter.

        Args:
            collection_name: Name of the collection
            key: Payload key to filter on
            match_value: Value to match for the key

        Returns:
            True if deletion was successful
        """
        try:
            # Create a filter to match the payload
            query_filter = models.Filter(
                must=[
                    models.FieldCondition(
                        key=key,
                        match=models.MatchValue(value=match_value)
                    )
                ]
            )

            # Delete points matching the filter
            self.client.delete(
                collection_name=collection_name,
                points_selector=models.FilterSelector(
                    filter=query_filter
                )
            )

            logger.info(f"Deleted points with {key}={match_value} from collection {collection_name}")
            return True

        except Exception as e:
            logger.error(f"Error deleting points by payload filter: {str(e)}")
            return False

    def create_payload_index(self,
                            collection_name: str,
                            field_name: str,
                            field_type: models.PayloadSchemaType = models.PayloadSchemaType.KEYWORD) -> bool:
        """
        Create an index on a payload field for faster filtering.

        Args:
            collection_name: Name of the collection
            field_name: Name of the field to index
            field_type: Type of the field

        Returns:
            True if index was created successfully
        """
        try:
            self.client.create_payload_index(
                collection_name=collection_name,
                field_name=field_name,
                field_schema=field_type
            )
            logger.info(f"Created payload index for {field_name} in collection {collection_name}")
            return True
        except Exception as e:
            logger.error(f"Error creating payload index: {str(e)}")
            return False

    def get_points_by_ids(self,
                         collection_name: str,
                         point_ids: List[str],
                         with_payload: bool = True,
                         with_vectors: bool = False) -> List[models.Record]:
        """
        Retrieve specific points by their IDs.

        Args:
            collection_name: Name of the collection
            point_ids: List of point IDs to retrieve
            with_payload: Whether to return payload data
            with_vectors: Whether to return vector data

        Returns:
            List of records matching the IDs
        """
        try:
            records = self.client.retrieve(
                collection_name=collection_name,
                ids=point_ids,
                with_payload=with_payload,
                with_vectors=with_vectors
            )
            return records
        except Exception as e:
            logger.error(f"Error retrieving points by IDs: {str(e)}")
            return []

# Global Qdrant client instance
qdrant_client: Optional[EnhancedQdrantClient] = None

def get_qdrant_client(url: str, api_key: str) -> EnhancedQdrantClient:
    """
    Get or create the global Qdrant client instance.

    Args:
        url: Qdrant server URL
        api_key: Qdrant API key

    Returns:
        EnhancedQdrantClient instance
    """
    global qdrant_client
    if qdrant_client is None:
        qdrant_client = EnhancedQdrantClient(url, api_key)
    return qdrant_client