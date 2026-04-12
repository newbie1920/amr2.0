import { createClient } from '@supabase/supabase-js'

export const supabaseUrl = 'https://tatsbivjghjjznkdqnde.supabase.co'
export const supabaseKey = 'eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiJzdXBhYmFzZSIsInJlZiI6InRhdHNiaXZqZ2hqanpua2RxbmRlIiwicm9sZSI6ImFub24iLCJpYXQiOjE3NzU4OTczNjUsImV4cCI6MjA5MTQ3MzM2NX0.nWhiCRvGH0_fcrNRlcLZdI0E6RBMwKw3QMy-kqrCxoA'
export const supabase = createClient(supabaseUrl, supabaseKey)
