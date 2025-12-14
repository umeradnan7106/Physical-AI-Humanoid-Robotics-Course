# RAG Chatbot Backend

AI-powered chatbot for Physical AI & Humanoid Robotics book using RAG (Retrieval-Augmented Generation).

## Features

- **Retrieval-Augmented Generation**: Answers based on book content
- **Selected Text Chat**: Ask questions about highlighted text
- **Conversation History**: Persistent chat sessions
- **Vector Search**: Semantic search using Qdrant
- **PostgreSQL Storage**: Chat history and analytics

## Tech Stack

- **Backend**: FastAPI (Python)
- **Vector DB**: Qdrant Cloud
- **SQL DB**: Neon Serverless Postgres
- **AI Model**: OpenAI GPT-4o-mini
- **Embeddings**: OpenAI text-embedding-3-small

## Setup Instructions

### 1. Install Dependencies

```bash
cd backend
pip install -r requirements.txt
```

### 2. Configure Environment Variables

Create `.env` file:

```bash
cp .env.example .env
```

Fill in your API keys:

```env
OPENAI_API_KEY=sk-your-key-here
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your-qdrant-key
DATABASE_URL=postgresql://user:pass@host.neon.tech/db?sslmode=require
```

### 3. Initialize Database

```bash
python main.py
```

This will create all necessary tables.

### 4. Ingest Book Content

```bash
python ingest.py
```

This will:
- Process all markdown files
- Generate embeddings
- Upload to Qdrant vector database

Expected output:
```
🚀 Starting book content ingestion...
Found 50 markdown files
Processing: ros2-architecture.md
  ✅ Uploaded 12 chunks
...
🎉 Ingestion complete!
📊 Total chunks uploaded: 450
```

### 5. Run Development Server

```bash
python main.py
```

or

```bash
uvicorn main:app --reload --port 8000
```

Server will run at: http://localhost:8000

### 6. Test API

Health check:
```bash
curl http://localhost:8000/health
```

Test chat:
```bash
curl -X POST http://localhost:8000/api/chat \
  -H "Content-Type: application/json" \
  -d '{"message": "What is ROS 2?"}'
```

## API Endpoints

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/` or `/health` | GET | Health check |
| `/api/chat` | POST | Normal chat |
| `/api/chat-selection` | POST | Chat with selected text |
| `/api/feedback` | POST | Submit feedback |
| `/api/history/{session_id}` | GET | Get chat history |

## Deployment

### Railway (Recommended)

1. Create Railway account
2. New Project → Deploy from GitHub
3. Add environment variables
4. Deploy!

### Vercel

```bash
pip install vercel
vercel --prod
```

### Docker

```dockerfile
FROM python:3.11-slim
WORKDIR /app
COPY requirements.txt .
RUN pip install -r requirements.txt
COPY . .
CMD ["uvicorn", "main:app", "--host", "0.0.0.0", "--port", "8000"]
```

## Cost Estimation

- **Neon Postgres**: Free (3GB)
- **Qdrant Cloud**: Free (1GB)
- **OpenAI API**: ~$5-10/month
  - Embeddings: $0.0001 per 1K tokens (~$0.50 for entire book)
  - GPT-4o-mini: $0.15 per 1M input tokens

## Troubleshooting

**Database connection error**:
```
Check DATABASE_URL format and SSL mode
```

**Qdrant connection error**:
```
Verify QDRANT_URL and QDRANT_API_KEY
```

**OpenAI rate limit**:
```
Add delays or upgrade to higher tier
```

## License

MIT
