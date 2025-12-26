"""
Load testing script for the RAG Chatbot API
This script tests the system under concurrent load to validate performance
"""
import asyncio
import time
import aiohttp
import json
from typing import List, Dict, Any
from dataclasses import dataclass
from concurrent.futures import ThreadPoolExecutor
import statistics

@dataclass
class LoadTestResult:
    """Data class to store load test results"""
    total_requests: int
    successful_requests: int
    failed_requests: int
    response_times: List[float]
    avg_response_time: float
    min_response_time: float
    max_response_time: float
    p95_response_time: float
    p99_response_time: float
    requests_per_second: float
    errors: List[str]


class LoadTester:
    """Load tester for the RAG Chatbot API"""

    def __init__(self, base_url: str, concurrency: int = 10):
        self.base_url = base_url
        self.concurrency = concurrency
        self.session = None

    async def __aenter__(self):
        self.session = aiohttp.ClientSession()
        return self

    async def __aexit__(self, exc_type, exc_val, exc_tb):
        if self.session:
            await self.session.close()

    async def make_request(self, query: str, session_id: str) -> Dict[str, Any]:
        """Make a single request to the API"""
        start_time = time.time()
        try:
            payload = {
                "query": query,
                "session_id": session_id,
                "include_citations": True
            }

            async with self.session.post(
                f"{self.base_url}/api/chat/query",
                json=payload,
                timeout=aiohttp.ClientTimeout(total=30)
            ) as response:
                response_time = (time.time() - start_time) * 1000  # Convert to milliseconds
                response_data = await response.json()

                return {
                    "status": response.status,
                    "response_time": response_time,
                    "data": response_data,
                    "error": None
                }
        except Exception as e:
            response_time = (time.time() - start_time) * 1000  # Convert to milliseconds
            return {
                "status": None,
                "response_time": response_time,
                "data": None,
                "error": str(e)
            }

    async def run_load_test(
        self,
        num_requests: int,
        test_duration_seconds: int = None,
        query: str = "What is Physical AI?"
    ) -> LoadTestResult:
        """Run a load test against the API"""
        print(f"Starting load test: {num_requests} requests with concurrency {self.concurrency}")

        start_time = time.time()
        tasks = []
        responses = []

        for i in range(num_requests):
            session_id = f"load-test-session-{i % 100}"  # Cycle through 100 session IDs
            task = self.make_request(query, session_id)
            tasks.append(task)

            # If we've reached concurrency limit, wait for some tasks to complete
            if len(tasks) >= self.concurrency:
                batch_responses = await asyncio.gather(*tasks, return_exceptions=True)
                responses.extend(batch_responses)
                tasks = []

        # Wait for remaining tasks
        if tasks:
            remaining_responses = await asyncio.gather(*tasks, return_exceptions=True)
            responses.extend(remaining_responses)

        end_time = time.time()
        total_time = end_time - start_time

        # Process results
        successful_requests = 0
        failed_requests = 0
        response_times = []
        errors = []

        for response in responses:
            if isinstance(response, Exception):
                failed_requests += 1
                errors.append(str(response))
                continue

            if response["error"]:
                failed_requests += 1
                errors.append(response["error"])
            else:
                if response["status"] == 200:
                    successful_requests += 1
                    response_times.append(response["response_time"])
                else:
                    failed_requests += 1
                    errors.append(f"HTTP {response['status']}: {response.get('data', 'Unknown error')}")

        # Calculate statistics
        if response_times:
            avg_response_time = statistics.mean(response_times)
            min_response_time = min(response_times)
            max_response_time = max(response_times)
            p95_response_time = statistics.quantiles(response_times, n=20)[18] if len(response_times) > 1 else response_times[0]  # 95th percentile
            p99_response_time = statistics.quantiles(response_times, n=100)[98] if len(response_times) > 1 else response_times[0]  # 99th percentile
        else:
            avg_response_time = min_response_time = max_response_time = p95_response_time = p99_response_time = 0

        requests_per_second = num_requests / total_time if total_time > 0 else 0

        result = LoadTestResult(
            total_requests=num_requests,
            successful_requests=successful_requests,
            failed_requests=failed_requests,
            response_times=response_times,
            avg_response_time=avg_response_time,
            min_response_time=min_response_time,
            max_response_time=max_response_time,
            p95_response_time=p95_response_time,
            p99_response_time=p99_response_time,
            requests_per_second=requests_per_second,
            errors=errors
        )

        return result


async def main():
    """Run the load test"""
    print("Starting RAG Chatbot API Load Test")
    print("=" * 50)

    # Configuration
    BASE_URL = "http://localhost:8000"  # Update this to your API endpoint
    NUM_REQUESTS = 50
    CONCURRENCY = 10

    async with LoadTester(BASE_URL, CONCURRENCY) as load_tester:
        result = await load_tester.run_load_test(NUM_REQUESTS)

        # Print results
        print(f"\nLoad Test Results:")
        print(f"Total Requests: {result.total_requests}")
        print(f"Successful Requests: {result.successful_requests}")
        print(f"Failed Requests: {result.failed_requests}")
        print(f"Success Rate: {(result.successful_requests / result.total_requests * 100):.2f}%")
        print(f"Requests/Second: {result.requests_per_second:.2f}")
        print(f"Avg Response Time: {result.avg_response_time:.2f}ms")
        print(f"Min Response Time: {result.min_response_time:.2f}ms")
        print(f"Max Response Time: {result.max_response_time:.2f}ms")
        print(f"95th Percentile: {result.p95_response_time:.2f}ms")
        print(f"99th Percentile: {result.p99_response_time:.2f}ms")

        if result.errors:
            print(f"\nErrors ({len(result.errors)}):")
            for error in result.errors[:10]:  # Show first 10 errors
                print(f"  - {error}")
            if len(result.errors) > 10:
                print(f"  ... and {len(result.errors) - 10} more errors")

        # Performance validation
        print(f"\nPerformance Validation:")
        if result.avg_response_time < 3000:  # Less than 3 seconds
            print("✅ Average response time under 3 seconds: PASSED")
        else:
            print("❌ Average response time under 3 seconds: FAILED")

        if result.p95_response_time < 5000:  # Less than 5 seconds for 95% of requests
            print("✅ 95th percentile response time under 5 seconds: PASSED")
        else:
            print("❌ 95th percentile response time under 5 seconds: FAILED")

        if result.success_rate >= 0.95:  # 95% success rate
            print("✅ Success rate >= 95%: PASSED")
        else:
            print("❌ Success rate >= 95%: FAILED")


if __name__ == "__main__":
    # Note: This would be run when the server is running
    # asyncio.run(main())
    print("Load testing script created. To run the test:")
    print("1. Start the backend server: uvicorn main:app --reload --port 8000")
    print("2. Run this script: python test_load.py")
    print("\nNote: The actual test execution is commented out to prevent running without a server.")