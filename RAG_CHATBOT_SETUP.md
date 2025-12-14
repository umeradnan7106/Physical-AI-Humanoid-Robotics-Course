# 🤖 RAG Chatbot Setup Guide

Complete guide to setup the AI-powered chatbot for your Physical AI & Humanoid Robotics book.

## 📋 What You'll Build

A production-ready RAG (Retrieval-Augmented Generation) chatbot that:
- ✅ Answers questions based on your book content
- ✅ Handles selected text queries
- ✅ Stores conversation history
- ✅ Provides source citations
- ✅ Works on GitHub Pages (live deployment)

## 🏗️ Architecture

```
User (GitHub Pages)
    ↓
Chatbot Widget (React)
    ↓
FastAPI Backend (Railway/Vercel)
    ↓
├── Qdrant (Vector Search)
├── OpenAI (Embeddings + Chat)
└── Neon Postgres (Chat History)
```

## 📝 Prerequisites

You need to create 3 accounts (all have free tiers):

### 1. OpenAI Account
1. Visit: https://platform.openai.com/signup
2. Add payment method (minimum $5)
3. Generate API key: https://platform.openai.com/api-keys
4. Copy key (starts with `sk-...`)

### 2. Neon Postgres
1. Visit: https://neon.tech
2. Sign up with GitHub
3. Create new project (select Free tier)
4. Copy connection string:
   ```
   postgresql://user:password@host.neon.tech/database?sslmode=require
   ```

### 3. Qdrant Cloud  
1. Visit: https://cloud.qdrant.io
2. Create account
3. Create cluster (Free tier - 1GB)
4. Copy:
   - Cluster URL: `https://xyz.qdrant.io`
   - API Key

## 🚀 Step-by-Step Setup

### Step 1: Install Python Dependencies

```bash
cd backend
pip install -r requirements.txt
```

### Step 2: Configure Environment

Create `backend/.env` file:

```env
# OpenAI
OPENAI_API_KEY=sk-proj-your-actual-key-here

# Qdrant
QDRANT_URL=https://your-cluster-id.qdrant.io
QDRANT_API_KEY=your-actual-qdrant-key
QDRANT_COLLECTION_NAME=physical_ai_book

# Neon Postgres
DATABASE_URL=postgresql://user:password@ep-xxx.us-east-2.aws.neon.tech/neondb?sslmode=require

# Server
PORT=8000
ENVIRONMENT=production
FRONTEND_URL=https://umeradnan7106.github.io
```

### Step 3: Initialize Database

```bash
cd backend
python main.py
```

You should see:
```
✅ Database tables created successfully!
```

### Step 4: Ingest Book Content (One-time)

This processes all your book chapters and uploads them to Qdrant:

```bash
cd backend
python ingest.py
```

Expected output:
```
🚀 Starting book content ingestion...
Found 50 markdown files
Processing: ros2-architecture.md
  ✅ Uploaded 12 chunks
Processing: gazebo-fundamentals.md
  ✅ Uploaded 8 chunks
...
🎉 Ingestion complete!
📊 Total chunks uploaded: 450
📚 Total files processed: 50
```

⏱️ This takes ~5-10 minutes depending on book size.

### Step 5: Test Locally

Start backend:
```bash
cd backend
python main.py
```

Backend runs at: http://localhost:8000

Test health:
```bash
curl http://localhost:8000/health
```

Test chat:
```bash
curl -X POST http://localhost:8000/api/chat \
  -H "Content-Type: application/json" \
  -d '{"message": "What is ROS 2?"}'
```

### Step 6: Deploy Backend to Production

#### Option A: Railway (Recommended - Easiest)

1. Visit: https://railway.app
2. Sign up with GitHub
3. New Project → Deploy from GitHub
4. Select your repository
5. Add environment variables (from your .env)
6. Deploy!

Railway will give you a URL like:
```
https://your-app.railway.app
```

#### Option B: Vercel

```bash
pip install vercel
vercel --prod
```

#### Option C: Render

1. Visit: https://render.com
2. New → Web Service
3. Connect GitHub repo
4. Build command: `pip install -r backend/requirements.txt`
5. Start command: `uvicorn backend.main:app --host 0.0.0.0 --port $PORT`

### Step 7: Add Frontend Widget

Create `.env.local` in project root:

```env
REACT_APP_API_URL=https://your-backend-url.railway.app
```

The chatbot widget will automatically connect to your backend!

### Step 8: Test on GitHub Pages

1. Commit all changes:
```bash
git add .
git commit -m "Add RAG chatbot with backend"
git push origin main
```

2. Wait for GitHub Actions to deploy

3. Visit your book:
```
https://umeradnan7106.github.io/Physical-AI-Humanoid-Robotics-Course/
```

4. Look for chatbot icon in bottom-right corner 🤖

## 🧪 Testing the Chatbot

Try these questions:
- "What is ROS 2?"
- "How do I create a URDF file?"
- "Explain Gazebo simulation"
- "What are the differences between Isaac Sim and Gazebo?"

## 💰 Cost Breakdown

### Free Forever:
- ✅ Neon Postgres: 3GB storage (enough)
- ✅ Qdrant Cloud: 1GB vectors (plenty for book)
- ✅ Railway/Vercel: Hobby tier (sufficient)

### Paid (Very Cheap):
- 💵 OpenAI API:
  - One-time embedding: ~$0.50 (entire book)
  - Chat usage: ~$5-10/month (500-1000 questions)

**Total: ~$5-10/month** for moderate usage

## 🐛 Troubleshooting

### "Database connection failed"
- Check DATABASE_URL format
- Ensure `?sslmode=require` is appended
- Test connection: `psql $DATABASE_URL`

### "Qdrant connection error"
- Verify QDRANT_URL (must include `https://`)
- Check API key is correct
- Ensure collection is created

### "OpenAI rate limit"
- Wait 1 minute
- Or upgrade to Tier 2 ($50 spend unlocks higher limits)

### "Chatbot not appearing"
- Check browser console for errors
- Verify REACT_APP_API_URL is set
- Test backend health endpoint

### "Empty responses"
- Run ingestion script again
- Check Qdrant has data:
  ```python
  from qdrant_client import QdrantClient
  client = QdrantClient(url="...", api_key="...")
  print(client.count(collection_name="physical_ai_book"))
  ```

## 📊 Monitoring Usage

Check OpenAI usage:
https://platform.openai.com/usage

Check Qdrant storage:
https://cloud.qdrant.io

Check Neon storage:
https://console.neon.tech

## 🎯 Next Steps

1. ✅ Setup all accounts
2. ✅ Configure .env file
3. ✅ Run ingestion script
4. ✅ Deploy backend
5. ✅ Test chatbot
6. ✅ Monitor usage

## 📞 Support

Issues? Check:
1. Backend logs
2. Browser console
3. API endpoint health
4. Environment variables

Happy chatting! 🚀
