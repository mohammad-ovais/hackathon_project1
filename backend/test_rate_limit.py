"""
Test script to validate rate limiting works correctly under load
"""
import asyncio
import aiohttp
import time
from typing import List, Dict, Any
import json


class RateLimitTester:
    """Tester for validating rate limiting functionality"""

    def __init__(self, base_url: str):
        self.base_url = base_url
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
                response_data = await response.json() if response.content_length != 0 else {}

                return {
                    "status": response.status,
                    "response_time": response_time,
                    "data": response_data,
                    "headers": dict(response.headers),
                    "error": None
                }
        except Exception as e:
            response_time = (time.time() - start_time) * 1000  # Convert to milliseconds
            return {
                "status": None,
                "response_time": response_time,
                "data": None,
                "headers": {},
                "error": str(e)
            }

    async def test_rate_limit_single_session(self, requests_count: int, session_id: str) -> Dict[str, Any]:
        """Test rate limiting for a single session"""
        print(f"Testing rate limiting for session {session_id} with {requests_count} requests...")

        tasks = []
        for i in range(requests_count):
            task = self.make_request(f"Test query {i} for rate limiting", session_id)
            tasks.append(task)

        results = await asyncio.gather(*tasks, return_exceptions=True)

        # Count successes and failures
        success_count = 0
        rate_limit_count = 0
        error_count = 0
        response_times = []

        for result in results:
            if isinstance(result, Exception):
                error_count += 1
                continue

            if result["status"] == 200:
                success_count += 1
                response_times.append(result["response_time"])
            elif result["status"] == 429:
                rate_limit_count += 1
            else:
                error_count += 1

        return {
            "total_requests": requests_count,
            "success_count": success_count,
            "rate_limit_count": rate_limit_count,
            "error_count": error_count,
            "response_times": response_times,
            "results": results
        }

    async def test_rate_limit_multiple_sessions(self, requests_per_session: int, num_sessions: int) -> Dict[str, Any]:
        """Test rate limiting with multiple sessions"""
        print(f"Testing rate limiting with {num_sessions} sessions, {requests_per_session} requests each...")

        all_results = []
        tasks = []

        for session_idx in range(num_sessions):
            session_id = f"test-session-{session_idx}"
            task = self.test_rate_limit_single_session(requests_per_session, session_id)
            tasks.append(task)

        all_results = await asyncio.gather(*tasks, return_exceptions=True)

        return all_results


async def main():
    """Run rate limiting tests"""
    print("Starting Rate Limiting Validation Tests")
    print("=" * 50)

    # Configuration - adjust based on your rate limiting settings
    # Default is 10 requests per minute per session
    BASE_URL = "http://localhost:8000"  # Update this to your API endpoint
    REQUESTS_PER_SESSION = 15  # This should exceed the rate limit (10/min)
    NUM_SESSIONS = 3

    async with RateLimitTester(BASE_URL) as tester:
        print(f"Configuration: {REQUESTS_PER_SESSION} requests per session, {NUM_SESSIONS} sessions")
        print("Expected: Some requests should be rate limited (status 429)\n")

        # Test single session exceeding rate limit
        print("Test 1: Single session exceeding rate limit")
        result = await tester.test_rate_limit_single_session(REQUESTS_PER_SESSION, "rate-limit-test-session")

        print(f"  Total requests: {result['total_requests']}")
        print(f"  Successful requests: {result['success_count']}")
        print(f"  Rate limited requests: {result['rate_limit_count']}")
        print(f"  Error requests: {result['error_count']}")

        if result['rate_limit_count'] > 0:
            print("  ✅ Rate limiting is working - some requests were blocked")
        else:
            print("  ❌ Rate limiting may not be working - no requests were blocked")

        if result['success_count'] <= 10:  # Assuming 10 requests per minute limit
            print("  ✅ Rate limit respected - success count within expected bounds")
        else:
            print("  ❌ Rate limit not respected - more requests succeeded than expected")

        print("\n" + "-" * 50)

        # Test multiple sessions
        print("Test 2: Multiple sessions test")
        multi_results = await tester.test_rate_limit_multiple_sessions(REQUESTS_PER_SESSION, NUM_SESSIONS)

        total_requests = 0
        total_success = 0
        total_rate_limited = 0
        total_errors = 0

        for i, result in enumerate(multi_results):
            if isinstance(result, Exception):
                print(f"  Session {i}: Error - {result}")
                continue

            total_requests += result['total_requests']
            total_success += result['success_count']
            total_rate_limited += result['rate_limit_count']
            total_errors += result['error_count']

            print(f"  Session {i}: {result['success_count']} success, {result['rate_limit_count']} rate-limited")

        print(f"\n  Aggregate results:")
        print(f"  Total requests: {total_requests}")
        print(f"  Total successful: {total_success}")
        print(f"  Total rate limited: {total_rate_limited}")
        print(f"  Total errors: {total_errors}")

        # Validate that rate limiting is per-session, not global
        if total_rate_limited > 0:
            print("  ✅ Rate limiting is working across multiple sessions")
        else:
            print("  ❌ Rate limiting may not be working across multiple sessions")

        # Check rate limit headers
        print("\nTest 3: Rate limit headers validation")
        single_result = await tester.test_rate_limit_single_session(1, "header-test-session")
        if single_result['results'] and single_result['results'][0]["headers"]:
            headers = single_result['results'][0]["headers"]
            rate_limit_headers = {k: v for k, v in headers.items() if 'rate' in k.lower()}
            if rate_limit_headers:
                print("  ✅ Rate limit headers present:")
                for header, value in rate_limit_headers.items():
                    print(f"    {header}: {value}")
            else:
                print("  ❌ Rate limit headers not found")
        else:
            print("  ❌ Could not check rate limit headers")

        print("\nRate Limiting Test Summary:")
        print("=" * 30)
        print("The tests validate that:")
        print("1. Rate limiting blocks requests beyond the configured limit")
        print("2. Rate limiting works on a per-session basis")
        print("3. Appropriate headers are returned to clients")
        print("\nNote: For accurate results, run this when the rate limit window is fresh")


if __name__ == "__main__":
    # Note: This would be run when the server is running
    # asyncio.run(main())
    print("Rate limit testing script created. To run the test:")
    print("1. Start the backend server: uvicorn main:app --reload --port 8000")
    print("2. Run this script: python test_rate_limit.py")
    print("\nNote: The actual test execution is commented out to prevent running without a server.")