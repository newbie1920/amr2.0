import { createClient } from '@supabase/supabase-js'

const supabaseUrl = import.meta.env.VITE_SUPABASE_URL
const supabaseKey = import.meta.env.VITE_SUPABASE_KEY

if (!supabaseUrl || !supabaseKey) {
  console.error('[Supabase] ❌ Missing VITE_SUPABASE_URL or VITE_SUPABASE_KEY in .env — Database features disabled!')
}

// FIX Bug #7: Don't create client with empty string (causes silent failures)
// Only create a real client if both URL and KEY are present
export const supabase = (supabaseUrl && supabaseKey)
  ? createClient(supabaseUrl, supabaseKey)
  : null
