# 🆓 FREE RAG Chatbot Setup Guide

**100% FREE** AI-powered chatbot for your Physical AI & Humanoid Robotics book!

## 💰 Total Cost: $0/month

This version uses completely FREE services:
- ✅ **Google Gemini** (FREE tier - no credit card required!)
- ✅ **Sentence Transformers** (100% local, runs on your computer)
- ✅ **Qdrant Cloud** (FREE 1GB tier)
- ✅ **Neon Postgres** (FREE 3GB tier)
- ✅ **Railway/Vercel** (FREE hobby tier)

## 📋 What You'll Build

A production-ready RAG chatbot that:
- ✅ Answers questions based on your book content
- ✅ Handles selected text queries
- ✅ Stores conversation history
- ✅ Provides source citations
- ✅ Works on GitHub Pages (live deployment)

## 🚀 Quick Start (3 Steps)

### Step 1: Create FREE Accounts (5 minutes)

#### 1.1 Google Gemini (No Credit Card!)
1. Visit: https://makersuite.google.com/app/apikey
2. Click "Create API Key"
3. Copy the key (starts with `AI...`)
4. **No credit card required!** 🎉

#### 1.2 Neon Postgres (FREE 3GB)
1. Visit: https://neon.tech
2. Sign up with GitHub
3. Create new project → Select **Free tier**
4. Copy connection string:
   ```
   postgresql://user:pass@ep-xxx.us-east-2.aws.neon.tech/neondb?sslmode=require
   ```

#### 1.3 Qdrant Cloud (FREE 1GB)
1. Visit: https://cloud.qdrant.io
2. Create account
3. Create cluster → Select **Free tier (1GB)**
4. Copy:
   - Cluster URL: `https://xxx-xxx-xxx.qdrant.io`
   - API Key

### Step 2: Install & Configure (2 minutes)

```bash
# Navigate to backend directory
cd backend

# Install dependencies (FREE local models included!)
pip install -r requirements_free.txt
```

**Create `.env` file** (copy from `.env.example`):

```bash
cp .env.example .env
```

**Fill in your FREE API keys** in `backend/.env`:

```env
# Google Gemini (FREE - get from https://makersuite.google.com/app/apikey)
GOOGLE_API_KEY=AIza...your-actual-key-here

# Qdrant Cloud (FREE 1GB)
QDRANT_URL=https://your-cluster-id.qdrant.io
QDRANT_API_KEY=your-actual-qdrant-key
QDRANT_COLLECTION_NAME=physical_ai_book

# Neon Postgres (FREE 3GB)
DATABASE_URL=postgresql://user:pass@ep-xxx.us-east-2.aws.neon.tech/neondb?sslmode=require

# Server
PORT=8000
FRONTEND_URL=https://yourusername.github.io
```

### Step 3: Run Setup (One-time)

```bash
# Initialize database
python main.py
```

You should see:
```
✅ Database initialized successfully!
```

Press `Ctrl+C` to stop.

```bash
# Ingest book content (one-time, ~5-10 minutes)
python ingest_free.py
```

Expected output:
```
🚀 Starting book content ingestion (FREE VERSION)...
📦 Using Sentence Transformers for embeddings (384 dimensions)
Found 50 markdown files
Processing: ros2-architecture.md
  ✅ Uploaded 12 chunks
Processing: gazebo-fundamentals.md
  ✅ Uploaded 8 chunks
...
🎉 Ingestion complete!
📊 Total chunks uploaded: 450
```

⏱️ **Note**: First run downloads the embedding model (~80MB) - subsequent runs are instant!

## ✅ Test Locally

Start the backend:
```bash
python main.py
```

Output:
```
INFO:     Uvicorn running on http://0.0.0.0:8000
✅ Database initialized successfully!
```

**Test in another terminal:**

```bash
# Health check
curl http://localhost:8000/health
```

Response:
```json
{
  "status": "healthy",
  "version": "1.0.0",
  "message": "RAG Chatbot API is running!"
}
```

**Test chat:**

```bash
curl -X POST http://localhost:8000/api/chat \
  -H "Content-Type: application/json" \
  -d "{\"message\": \"What is ROS 2?\"}"
```

You should get an AI-generated answer with sources!

## 🌐 Deploy Backend (FREE)

### Option A: Railway (Recommended - Easiest)

1. Visit: https://railway.app
2. Sign up with GitHub (FREE)
3. **New Project** → **Deploy from GitHub**
4. Select your repository
5. **Add Variables** (copy from your `.env` file):
   - `GOOGLE_API_KEY`
   - `QDRANT_URL`
   - `QDRANT_API_KEY`
   - `QDRANT_COLLECTION_NAME`
   - `DATABASE_URL`
   - `FRONTEND_URL`
6. Click **Deploy**

Railway will give you a FREE URL:
```
https://your-app.railway.app
```

### Option B: Vercel (FREE)

```bash
pip install vercel
vercel --prod
```

Follow prompts and add environment variables.

## 🎨 Add Frontend Widget

Create `frontend/.env.local`:

```env
REACT_APP_API_URL=https://your-backend.railway.app
```

The chatbot widget will automatically connect!

## 🧪 Test Sample Questions

Try these in your chatbot:
- "What is ROS 2?"
- "How do I create a URDF file?"
- "Explain Gazebo simulation"
- "What are the differences between Isaac Sim and Gazebo?"
- "Show me how to launch a ROS 2 node"

## 📊 What's Happening Behind the Scenes?

### 🔍 When You Ask a Question:

1. **Your question** → Converted to a 384-dimensional vector using **Sentence Transformers** (runs locally on your machine!)
2. **Vector search** → Qdrant finds the 3 most similar chunks from your book
3. **Context building** → Relevant book excerpts are gathered
4. **AI generation** → Google Gemini generates an answer based on the context
5. **Response** → Answer + source citations returned to you

### 💾 Data Storage:

- **Qdrant**: Stores book content as vectors (384 dimensions)
- **Neon Postgres**: Stores chat history, feedback, analytics
- **Local**: Embedding model cached on your machine (~80MB)

## 🔧 Troubleshooting

### "Database connection failed"
```bash
# Check your DATABASE_URL format
# Must end with ?sslmode=require

# Test connection
python -c "from database import init_db; init_db()"
```

### "Qdrant connection error"
```bash
# Verify QDRANT_URL (must include https://)
# Check API key is correct

# Test connection
python -c "from rag_free import create_collection; create_collection()"
```

### "Google Gemini API error"
```bash
# Check your GOOGLE_API_KEY is correct
# Verify you're not hitting rate limits (60 requests/minute on free tier)

# Test API key
python -c "import google.generativeai as genai; import os; from dotenv import load_dotenv; load_dotenv(); genai.configure(api_key=os.getenv('GOOGLE_API_KEY')); print('API key works!')"
```

### "Embedding model download stuck"
```bash
# The first run downloads ~80MB model
# Check your internet connection
# Wait 2-3 minutes for download to complete
```

### "Empty responses from chatbot"
```bash
# Make sure you ran ingestion
python ingest_free.py

# Check Qdrant has data
python -c "from qdrant_client import QdrantClient; import os; from dotenv import load_dotenv; load_dotenv(); client = QdrantClient(url=os.getenv('QDRANT_URL'), api_key=os.getenv('QDRANT_API_KEY')); print(client.count(collection_name='physical_ai_book'))"
```

## 📈 Rate Limits (FREE Tier)

### Google Gemini:
- ✅ 60 requests per minute
- ✅ 1500 requests per day
- ✅ Sufficient for hundreds of questions/day

### Qdrant Cloud:
- ✅ 1GB storage (enough for entire book + more)
- ✅ Unlimited queries

### Neon Postgres:
- ✅ 3GB storage
- ✅ Unlimited queries
- ✅ Stores ~10,000+ conversations easily

## 🎯 Next Steps

1. ✅ Get your 3 FREE API keys
2. ✅ Configure `.env` file
3. ✅ Run `python ingest_free.py` (one-time)
4. ✅ Test locally with `python main.py`
5. ✅ Deploy to Railway (FREE)
6. ✅ Test on GitHub Pages

## 💡 Tips

- **First-time setup**: Takes ~10 minutes
- **Embedding model**: Downloads once, cached forever
- **Ingestion**: Run once per book update
- **Gemini API**: 1500 free requests/day = plenty for students
- **Cost**: $0/month forever! 🎉

## 📝 API Endpoints

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/health` | GET | Health check |
| `/api/chat` | POST | Normal chat |
| `/api/chat-selection` | POST | Chat with selected text |
| `/api/feedback` | POST | Submit feedback |
| `/api/history/{session_id}` | GET | Get chat history |

## 🔐 Security Notes

- ⚠️ **NEVER** commit `.env` file
- ✅ Use `.env.example` as template
- ✅ Add `.env` to `.gitignore`
- ✅ All API keys are FREE but keep them private

## 🆘 Need Help?

**Common Issues:**
1. Database connection → Check `?sslmode=require` in DATABASE_URL
2. Qdrant errors → Verify URL includes `https://`
3. Gemini errors → Confirm API key from https://makersuite.google.com
4. Empty responses → Re-run `python ingest_free.py`

**Still stuck?** Check:
- Backend logs for errors
- Browser console (F12)
- `/health` endpoint status
- All environment variables set correctly

---

## 🎊 Congratulations!

You now have a **FREE, production-ready AI chatbot** for your book!

**What you achieved:**
- ✅ 100% FREE RAG chatbot
- ✅ Local embeddings (no API costs)
- ✅ Google Gemini integration
- ✅ Vector search with Qdrant
- ✅ Chat history storage
- ✅ Live on GitHub Pages

**Total cost: $0/month** 🚀
