# Production Deployment Guide: RAG Chatbot

## Overview

This guide provides step-by-step instructions for deploying the RAG Chatbot in a production environment. The system is designed to operate within free-tier limits while providing reliable service.

## Prerequisites

### Required Services
- **Qdrant Cloud Account** (Free tier with 1GB storage)
- **Google AI Studio Account** (Free tier with Gemini API access)
- **Docusaurus Site** (for frontend integration)
- **Hosting Platform** (Railway, Render, Vercel, or self-hosted server)

### Required Tools
- Docker and Docker Compose
- Git
- Node.js 18+ (for Docusaurus build)
- Python 3.11+ (for backend deployment)

## Pre-Deployment Configuration

### 1. Obtain API Credentials

#### Gemini API Key
1. Go to [Google AI Studio](https://aistudio.google.com/)
2. Create an account or sign in
3. Click "Get API Key" for Gemini API
4. Save the API key securely

#### Qdrant Cloud Setup
1. Sign up at [Qdrant Cloud](https://cloud.qdrant.io/)
2. Create a new cluster
3. Note down the URL and API key
4. Plan for 1GB storage limit (free tier)

### 2. Environment Configuration

Create a `.env.production` file with the following content:

```env
# Gemini API Configuration
GEMINI_API_KEY=your_production_gemini_api_key

# Qdrant Configuration
QDRANT_URL=your_production_qdrant_url
QDRANT_API_KEY=your_production_qdrant_api_key
QDRANT_COLLECTION_NAME=textbook_chunks

# Application Configuration
LOG_LEVEL=INFO
WORKERS_COUNT=4
TIMEOUT=300
</pre>

## Deployment Options

### Option 1: Docker Compose Deployment

#### Step 1: Prepare the Codebase
```bash
# Clone the repository
git clone <your-repository-url>
cd <repository-directory>

# Navigate to backend directory
cd backend

# Create production environment file
cp .env.production .env
```

#### Step 2: Build the Ingestion Data
```bash
# Process textbook content (run once before deployment)
cd ingestion
source venv/bin/activate
python src/ingestion_pipeline.py --source-path ../docusaurus/docs --chunk-size 500 --overlap 50
```

#### Step 3: Deploy with Docker Compose
```bash
# From the root directory
docker-compose -f docker-compose.prod.yml up -d --build
```

### Option 2: Platform-as-a-Service (Recommended)

#### Railway Deployment

1. **Install Railway CLI**
   ```bash
   npm install -g @railway/cli
   ```

2. **Login to Railway**
   ```bash
   railway login
   ```

3. **Initialize Project**
   ```bash
   railway init
   ```

4. **Configure Variables**
   ```bash
   railway variables set GEMINI_API_KEY "your-key"
   railway variables set QDRANT_URL "your-url"
   railway variables set QDRANT_API_KEY "your-key"
   ```

5. **Deploy**
   ```bash
   railway up
   ```

#### Vercel/Render Deployment

Similar process with platform-specific configuration files.

## Frontend Integration

### Docusaurus Plugin Integration

The chatbot widget is integrated via the Docusaurus plugin. Ensure your `docusaurus.config.js` includes:

```js
module.exports = {
  // ... other config
  plugins: [
    [
      './src/plugins/chatbot-plugin.js',
      {
        backendUrl: process.env.BACKEND_URL || 'https://your-backend-domain.com',
        theme: 'light',
        position: 'right'
      }
    ]
  ],
  // ... rest of config
};
```

### Build and Deploy Docusaurus

```bash
# Install dependencies
npm install

# Build the site
npm run build

# Deploy to your hosting service
# (GitHub Pages, Netlify, Vercel, etc.)
```

## Post-Deployment Tasks

### 1. Verify Service Health

Check the health endpoint:
```bash
curl https://your-deployed-domain.com/api/health
```

Expected response:
```json
{
  "status": "healthy",
  "timestamp": "2025-12-26T00:00:00Z",
  "services": {
    "qdrant": "healthy",
    "gemini": "reachable"
  }
}
```

### 2. Test Functionality

Test the chat endpoint:
```bash
curl -X POST https://your-deployed-domain.com/api/chat/query \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What is Physical AI?",
    "session_id": "test-deployment-123",
    "include_citations": true
  }'
```

### 3. Monitor Resource Usage

- **Qdrant Storage**: Monitor to stay within 1GB limit
- **Gemini API Quotas**: Track usage against monthly limits
- **Response Times**: Ensure consistently under 3 seconds

## Scaling Considerations

### Horizontal Scaling
- Multiple backend instances behind a load balancer
- Shared Qdrant instance for vector storage
- Session-based routing for conversation continuity

### Performance Optimization
- Increase worker count in production
- Optimize chunk size based on content
- Implement CDN for static assets

### Rate Limiting
- Default: 10 requests per minute per session
- Adjust based on your API quota and user base
- Monitor for abuse patterns

## Monitoring and Maintenance

### Health Checks
- Set up automated health checks to `/api/health`
- Monitor response times and error rates
- Alert on service degradation

### Logging
- Monitor application logs for errors
- Track API usage patterns
- Watch for security incidents

### Backup Strategy
- Qdrant Cloud handles vector database backups
- Version control for code and configuration
- Document deployment procedures

## Troubleshooting

### Common Issues

#### Rate Limiting
- **Symptom**: 429 responses
- **Solution**: Verify rate limit configuration, check if legitimate traffic or abuse

#### Slow Responses
- **Symptom**: Response times > 3 seconds
- **Solution**: Check Qdrant/Gemini API health, consider caching optimization

#### Vector Database Connection
- **Symptom**: 500 errors related to Qdrant
- **Solution**: Verify URL and API key, check network connectivity

### Performance Tuning
- Adjust worker count based on traffic
- Tune chunk size for optimal retrieval
- Monitor and optimize embedding cache hit rates

## Security Best Practices

- Never expose API keys in client-side code
- Use HTTPS for all communications
- Implement proper input validation
- Monitor for unusual traffic patterns
- Regular security audits of dependencies

## Rollback Procedure

In case of deployment issues:

1. **Immediate Response**: Switch traffic to backup instance if available
2. **Rollback**: Deploy previous stable version
3. **Investigation**: Analyze logs to identify root cause
4. **Fix**: Address the issue in a development environment
5. **Test**: Verify fix in staging environment
6. **Redeploy**: Deploy to production with monitoring

## Conclusion

Your RAG Chatbot should now be successfully deployed in production. Remember to:

- Monitor resource usage regularly
- Maintain API key security
- Plan for scaling as usage grows
- Keep dependencies updated
- Document any customizations made during deployment

For ongoing maintenance, regularly check the application logs and monitor the health of integrated services (Qdrant, Gemini API).